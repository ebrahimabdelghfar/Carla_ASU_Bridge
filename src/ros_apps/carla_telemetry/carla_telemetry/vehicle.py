"""
| File: vehicle.py
| Description: CarlaVehicle — CARLA client connection and ego vehicle management.
               Handles:
                 - Connecting to the CARLA server
                 - Loading the configured town/map
                 - Spawning the ego vehicle blueprint at the configured spawn point
                 - Optionally launching manual_control.py as a subprocess
                 - Providing vehicle state (velocity, transform) for sensors
"""
__all__ = ["CarlaVehicle"]

import logging
import math
import os
import subprocess
import sys
import time

logger = logging.getLogger("carla_telemetry.vehicle")


class CarlaVehicle:
    """CARLA connection + ego vehicle lifecycle manager.

    Args:
        config: Full carla_interface_config.yaml dict (carla + world + vehicle sections).
    """

    def __init__(self, config: dict):
        import carla
        self._carla = carla

        carla_cfg   = config.get("carla", {})
        world_cfg   = config.get("world", {})
        vehicle_cfg = config.get("vehicle", {})

        host    = str(carla_cfg.get("host", "localhost"))
        port    = int(carla_cfg.get("port", 2000))
        timeout = float(carla_cfg.get("timeout", 10.0))

        logger.info("[CarlaVehicle] Connecting to CARLA at %s:%d …", host, port)
        self._client = carla.Client(host, port)
        self._client.set_timeout(timeout)

        # ── Load town ─────────────────────────────────────────────────
        town = str(world_cfg.get("town", "Town01"))
        current_map = self._client.get_world().get_map().name.split("/")[-1]
        if current_map != town:
            logger.info("[CarlaVehicle] Loading map '%s' …", town)
            self._client.load_world(town)
            time.sleep(2.0)   # let Unreal finish the map load

        self._world = self._client.get_world()

        # ── Synchronous mode ──────────────────────────────────────────
        self._original_settings = self._world.get_settings()
        self._sync_mode = bool(carla_cfg.get("synchronous_mode", True))

        if self._sync_mode:
            settings = self._world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(
                carla_cfg.get("fixed_delta_seconds", 0.05)
            )
            self._world.apply_settings(settings)

            tm = self._client.get_trafficmanager()
            tm.set_synchronous_mode(True)
            logger.info(
                "[CarlaVehicle] Synchronous mode enabled (%.3f s/tick).",
                carla_cfg.get("fixed_delta_seconds", 0.05),
            )

        # ── Spawn vehicle ─────────────────────────────────────────────
        blueprint_filter = str(vehicle_cfg.get("blueprint", "vehicle.lincoln.mkz_2017")).strip()
        if not blueprint_filter:
            blueprint_filter = "vehicle.lincoln.mkz_2017"

        role_name = str(vehicle_cfg.get("role_name", "hero")).strip()
        if not role_name:
            role_name = "hero"

        generation = str(vehicle_cfg.get("generation", "2")).strip()
        if not generation:
            generation = "2"

        bp_lib = self._world.get_blueprint_library()
        bps    = bp_lib.filter(blueprint_filter)
        if not bps:
            raise RuntimeError(f"No blueprints match filter '{blueprint_filter}'")

        # Filter by generation
        if generation.lower() not in ("all", ""):
            try:
                gen_int = int(generation)
                bps_filtered = [b for b in bps
                                if b.has_attribute("generation")
                                and int(b.get_attribute("generation")) == gen_int]
                if bps_filtered:
                    bps = bps_filtered
            except (ValueError, RuntimeError):
                pass

        blueprint = bps[0]
        blueprint.set_attribute("role_name", role_name)
        if blueprint.has_attribute("ros_name"):
            blueprint.set_attribute("ros_name", role_name)

        # Color
        color_cfg = vehicle_cfg.get("color")
        if color_cfg and blueprint.has_attribute("color"):
            blueprint.set_attribute("color", str(color_cfg))
        elif blueprint.has_attribute("color"):
            rec = blueprint.get_attribute("color").recommended_values
            if rec:
                blueprint.set_attribute("color", rec[0])

        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "true")

        # Spawn point
        spawn_points = self._world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available in the map.")

        sp_idx = int(world_cfg.get("spawn_point_index", 0))
        if sp_idx < 0 or sp_idx >= len(spawn_points):
            import random
            spawn_point = random.choice(spawn_points)
            logger.warning(
                "[CarlaVehicle] spawn_point_index %d out of range [0, %d). "
                "Using random spawn.", sp_idx, len(spawn_points)
            )
        else:
            spawn_point = spawn_points[sp_idx]

        self._vehicle = None
        attempts = 5
        for i in range(attempts):
            self._vehicle = self._world.try_spawn_actor(blueprint, spawn_point)
            if self._vehicle is not None:
                break
            logger.warning("[CarlaVehicle] Spawn attempt %d failed. Retrying…", i + 1)
            time.sleep(0.5)

        if self._vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle after multiple attempts.")

        logger.info(
            "[CarlaVehicle] Spawned '%s' (role='%s') at spawn_point %d.",
            blueprint_filter, role_name, sp_idx,
        )

        if bool(vehicle_cfg.get("autopilot", False)):
            self._vehicle.set_autopilot(True)
            logger.info("[CarlaVehicle] Autopilot enabled.")

        # Apply physics control (transmission, drive mode, engine, wheels)
        self._apply_physics_control(vehicle_cfg)

        # ── Manual control subprocess ─────────────────────────────────
        self._manual_proc: subprocess.Popen | None = None
        if bool(carla_cfg.get("open_manual_control", False)):
            tx_cfg = vehicle_cfg.get("transmission", {})
            start_manual_gear = str(tx_cfg.get("type", "automatic")).lower() == "manual"
            self._launch_manual_control(
                host,
                port,
                blueprint_filter=blueprint_filter,
                generation=generation,
                role_name=role_name,
                start_manual_gear=start_manual_gear,
            )

    # ------------------------------------------------------------------
    # Getters
    # ------------------------------------------------------------------

    @property
    def world(self):
        """carla.World instance."""
        return self._world

    @property
    def actor(self):
        """The ego vehicle carla.Actor."""
        return self._vehicle

    @property
    def client(self):
        return self._client

    def get_velocity(self):
        """Return current vehicle velocity as carla.Vector3D (m/s)."""
        return self._vehicle.get_velocity()

    def get_transform(self):
        """Return current vehicle transform as carla.Transform."""
        return self._vehicle.get_transform()

    def get_speed_ms(self) -> float:
        """3D speed magnitude in m/s."""
        v = self._vehicle.get_velocity()
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def tick(self):
        """Advance one simulation tick (synchronous mode only)."""
        if self._sync_mode:
            self._world.tick()

    # ------------------------------------------------------------------
    # Physics control
    # ------------------------------------------------------------------

    def _apply_physics_control(self, vehicle_cfg: dict):
        """Apply VehiclePhysicsControl from the vehicle config section.

        Called once after the vehicle spawns.  Safely skipped if
        ``physics.enabled`` is False or the section is missing.
        """
        carla = self._carla

        physics_cfg = vehicle_cfg.get("physics", {})
        if not bool(physics_cfg.get("enabled", True)):
            logger.info("[CarlaVehicle] Physics control disabled — using blueprint defaults.")
            return

        # Start from the current (blueprint) physics as a baseline
        ctrl = self._vehicle.get_physics_control()

        # ── Engine & mass ─────────────────────────────────────────────
        if "mass" in physics_cfg:
            ctrl.mass = float(physics_cfg["mass"])
        if "drag_coefficient" in physics_cfg:
            ctrl.drag_coefficient = float(physics_cfg["drag_coefficient"])
        if "max_rpm" in physics_cfg:
            ctrl.max_rpm = float(physics_cfg["max_rpm"])
        if "moi" in physics_cfg:
            ctrl.moi = float(physics_cfg["moi"])
        if "damping_rate_full_throttle" in physics_cfg:
            ctrl.damping_rate_full_throttle = float(physics_cfg["damping_rate_full_throttle"])
        if "damping_rate_zero_throttle_clutch_engaged" in physics_cfg:
            ctrl.damping_rate_zero_throttle_clutch_engaged = float(
                physics_cfg["damping_rate_zero_throttle_clutch_engaged"]
            )
        if "damping_rate_zero_throttle_clutch_disengaged" in physics_cfg:
            ctrl.damping_rate_zero_throttle_clutch_disengaged = float(
                physics_cfg["damping_rate_zero_throttle_clutch_disengaged"]
            )
        if "use_sweep_wheel_collision" in physics_cfg:
            ctrl.use_sweep_wheel_collision = bool(physics_cfg["use_sweep_wheel_collision"])

        com = physics_cfg.get("center_of_mass")
        if com:
            ctrl.center_of_mass = carla.Vector3D(
                float(com.get("x", 0.0)),
                float(com.get("y", 0.0)),
                float(com.get("z", 0.0)),
            )

        # ── Torque curve ──────────────────────────────────────────────
        if "torque_curve" in physics_cfg:
            ctrl.torque_curve = [
                carla.Vector2D(float(pt[0]), float(pt[1]))
                for pt in physics_cfg["torque_curve"]
            ]

        # ── Steering curve ────────────────────────────────────────────
        if "steering_curve" in physics_cfg:
            ctrl.steering_curve = [
                carla.Vector2D(float(pt[0]), float(pt[1]))
                for pt in physics_cfg["steering_curve"]
            ]

        # ── Transmission ──────────────────────────────────────────────
        tx_cfg = vehicle_cfg.get("transmission", {})
        if tx_cfg:
            is_auto = str(tx_cfg.get("type", "automatic")).lower() == "automatic"
            ctrl.use_gear_autobox = is_auto
            if "gear_switch_time" in tx_cfg:
                ctrl.gear_switch_time = float(tx_cfg["gear_switch_time"])
            if "clutch_strength" in tx_cfg:
                ctrl.clutch_strength = float(tx_cfg["clutch_strength"])
            if "final_ratio" in tx_cfg:
                ctrl.final_ratio = float(tx_cfg["final_ratio"])
            gear_list = tx_cfg.get("forward_gears", [])
            if gear_list:
                ctrl.forward_gears = [
                    carla.GearPhysicsControl(
                        ratio=float(g["ratio"]),
                        down_ratio=float(g.get("down_ratio", 0.5)),
                        up_ratio=float(g.get("up_ratio", 0.65)),
                    )
                    for g in gear_list
                ]
            logger.info(
                "[CarlaVehicle] Transmission: %s, gear_switch_time=%.2f s, %d gears.",
                "automatic" if is_auto else "manual",
                ctrl.gear_switch_time,
                len(ctrl.forward_gears),
            )

        # ── Per-wheel physics (with drive_mode override) ───────────────
        wheels_cfg = physics_cfg.get("wheels", [])
        drive_mode = str(vehicle_cfg.get("drive_mode", "AWD")).upper()

        if wheels_cfg and len(ctrl.wheels) >= 4:
            # Build position → index map from the existing wheel list.
            # CARLA order: 0=FL, 1=FR, 2=RL, 3=RR
            pos_to_idx = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
            new_wheels = list(ctrl.wheels)   # copy list

            for wh_cfg in wheels_cfg:
                pos_label = str(wh_cfg.get("position", "")).upper()
                idx = pos_to_idx.get(pos_label)
                if idx is None:
                    continue
                w = new_wheels[idx]
                if "tire_friction"       in wh_cfg: w.tire_friction       = float(wh_cfg["tire_friction"])
                if "damping_rate"        in wh_cfg: w.damping_rate        = float(wh_cfg["damping_rate"])
                if "max_steer_angle"     in wh_cfg: w.max_steer_angle     = float(wh_cfg["max_steer_angle"])
                if "radius"              in wh_cfg: w.radius              = float(wh_cfg["radius"])
                if "max_brake_torque"    in wh_cfg: w.max_brake_torque    = float(wh_cfg["max_brake_torque"])
                if "max_handbrake_torque" in wh_cfg: w.max_handbrake_torque = float(wh_cfg["max_handbrake_torque"])

            # Drive mode: mute unpowered axle by setting near-zero friction
            _UNPOWERED_FRICTION = 0.1
            if drive_mode == "FWD":
                new_wheels[2].tire_friction = _UNPOWERED_FRICTION   # RL
                new_wheels[3].tire_friction = _UNPOWERED_FRICTION   # RR
            elif drive_mode == "RWD":
                new_wheels[0].tire_friction = _UNPOWERED_FRICTION   # FL
                new_wheels[1].tire_friction = _UNPOWERED_FRICTION   # FR
            # AWD: leave all wheels as configured

            ctrl.wheels = new_wheels
            logger.info("[CarlaVehicle] Drive mode: %s.", drive_mode)

        self._vehicle.apply_physics_control(ctrl)
        logger.info("[CarlaVehicle] Physics control applied.")

    # ------------------------------------------------------------------
    # Manual control subprocess
    # ------------------------------------------------------------------

    def _launch_manual_control(
        self,
        host: str,
        port: int,
        *,
        blueprint_filter: str,
        generation: str,
        role_name: str,
        start_manual_gear: bool = False,
    ):
        """Spawn manual_control.py as a detached subprocess.

        Uses:
          Bundled copy inside carla_telemetry package
          (``carla_telemetry/manual_control.py`` next to this file)
        """
        script_path = os.path.join(os.path.dirname(__file__), "manual_control.py")
        if not os.path.isfile(script_path):
            logger.error(
                "[CarlaVehicle] manual_control.py not found at '%s'. Skipping.",
                script_path,
            )
            return

        cmd = [
            sys.executable,
            script_path,
            "--host",
            host,
            "-p",
            str(port),
            "--rolename",
            role_name,
            "--filter",
            blueprint_filter,
            "--generation",
            generation,
            "--attach",
        ]
        if start_manual_gear:
            cmd.append("--start-manual-gear")
        try:
            self._manual_proc = subprocess.Popen(cmd)
            logger.info(
                "[CarlaVehicle] Launched manual_control.py (PID %d).",
                self._manual_proc.pid,
            )
        except Exception as exc:
            logger.error("[CarlaVehicle] Failed to launch manual_control.py: %s", exc)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self):
        """Restore world settings and destroy actors."""
        if self._manual_proc is not None:
            try:
                self._manual_proc.terminate()
            except Exception:
                pass
            self._manual_proc = None

        if self._vehicle is not None:
            try:
                self._vehicle.destroy()
            except Exception:
                pass
            self._vehicle = None

        # Restore original world settings (async mode, etc.)
        try:
            self._world.apply_settings(self._original_settings)
        except Exception:
            pass

        logger.info("[CarlaVehicle] Destroyed.")
