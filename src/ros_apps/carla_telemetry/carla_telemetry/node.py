"""
| File: node.py
| Description: CarlaTelemetryNode — the main ROS 2 node for CARLA telemetry.

               Startup sequence:
                 1. Load carla_interface_config.yaml
                 2. Connect to CARLA + spawn ego vehicle (CarlaVehicle)
                 3. Create CarlaROS2Backend (publishers + services)
                 4. Spawn GPS, Battery, IMU sensors
                 5. Create CarlaSensorManager → spawn cameras + lidars
                 6. Start sensor publish threads
                 7. Start ROS 2 timer loop (publish GPS + battery + IMU)
                 8. In sync mode, tick the world each timer callback

               Shutdown:
                 - Gracefully destroys all CARLA actors and stops threads

Usage:
    ros2 run carla_telemetry carla_telemetry_node \\
        --ros-args -p config_file:=/path/to/carla_interface_config.yaml

Or via launch file:
    ros2 launch carla_telemetry carla_telemetry.launch.py \\
        config_file:=/path/to/carla_interface_config.yaml
"""

import logging
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import yaml

from carla_telemetry.perf_monitor import perf

logger = logging.getLogger("carla_telemetry.node")


class CarlaTelemetryNode(Node):
    """Main ROS 2 node for CARLA telemetry.

    Parameters (ROS 2):
        config_file (string): Absolute path to carla_interface_config.yaml.
    """

    def __init__(self):
        super().__init__("micropilot_carla_bridge_node")

        # ── Declare ROS 2 parameters ─────────────────────────────────
        self.declare_parameter(
            "config_file",
            "",
            ParameterDescriptor(description="Absolute path to carla_interface_config.yaml"),
        )
        self.declare_parameter(
            "open_manual_control",
            "",
            ParameterDescriptor(description="Override for opening manual control (true/false)"),
        )

        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        # Fallback: look for carla_interface_config.yaml in workspace config/
        if not config_file or not os.path.isfile(config_file):
            project_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
            )
            default_path = os.path.join(
                project_root, "config", "carla_interface_config.yaml"
            )
            if os.path.isfile(default_path):
                config_file = os.path.abspath(default_path)
                self.get_logger().info(
                    f"config_file parameter not set. Using default: {config_file}"
                )
            else:
                self.get_logger().error(
                    "carla_interface_config.yaml not found. Set the 'config_file' parameter."
                )
                raise RuntimeError("carla_interface_config.yaml not found.")

        with open(config_file, "r") as f:
            self._cfg = yaml.safe_load(f)

        # self.get_logger().info("Loaded config from '%s'.", config_file)

        open_manual_control_str = self.get_parameter("open_manual_control").get_parameter_value().string_value
        if open_manual_control_str:
            val = open_manual_control_str.lower() in ("true", "1", "yes", "t")
            if "carla" not in self._cfg:
                self._cfg["carla"] = {}
            self._cfg["carla"]["open_manual_control"] = val
            self.get_logger().info(f"Overriding open_manual_control via param to: {val}")

        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        # ── Internal state ────────────────────────────────────────────
        self._vehicle       = None
        self._ros2_backend  = None
        self._gps           = None
        self._battery       = None
        self._imu           = None
        self._odometry      = None
        self._sensor_mgr    = None
        self._prev_tick_time = time.monotonic()
        self._prev_bat_time  = time.monotonic()

        self._tick_cb_group = MutuallyExclusiveCallbackGroup()
        self._shutdown_event = threading.Event()
        self._sensor_threads = []

        # ── Start everything ──────────────────────────────────────────
        try:
            self._startup()
        except Exception as exc:
            # self.get_logger().error("Startup failed: %s", exc)
            self._teardown()
            raise

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _startup(self):
        from carla_telemetry.vehicle import CarlaVehicle
        from carla_telemetry.sensors.gps import CarlaGPS
        from carla_telemetry.sensors.battery import CarlaBattery
        from carla_telemetry.sensors.imu import CarlaIMU
        from carla_telemetry.sensors.odometry import CarlaOdometry
        from carla_telemetry.sensor_manager import CarlaSensorManager
        from carla_telemetry.backends.ros2_backend import CarlaROS2Backend

        cfg = self._cfg

        # 1. Connect + spawn vehicle
        self._vehicle = CarlaVehicle(cfg)
        world  = self._vehicle.world
        actor  = self._vehicle.actor

        # 2. ROS 2 backend (pass vehicle so control callbacks can apply immediately)
        ros2_cfg = cfg.get("ros2", {})
        self._ros2_backend = CarlaROS2Backend(self, ros2_cfg, vehicle=self._vehicle)

        # Configure RPM → throttle and steer scale from control config section
        ctrl_cfg = cfg.get("control", {})
        self._ros2_backend.set_control_config(
            max_rpm=float(ctrl_cfg.get("max_rpm", 150.0)),
            max_steer_deg=float(ctrl_cfg.get("max_steer_deg", 70.0)),
        )

        # 3. GPS sensor
        gps_cfg = cfg.get("gps", {})
        origin = cfg.get("global_coordinates", {})
        self._gps = CarlaGPS(
            world, actor, gps_cfg,
            origin_lat=float(origin.get("latitude", 0.0)),
            origin_lon=float(origin.get("longitude", 0.0)),
            origin_alt=float(origin.get("altitude", 0.0)),
        )

        # 4. Battery model
        battery_cfg = cfg.get("battery", {})
        self._battery = CarlaBattery(battery_cfg)
        self._ros2_backend.set_battery_controller(self._battery)

        # 5. IMU sensor (optional)
        imu_cfg = cfg.get("imu", {})
        if bool(imu_cfg.get("enabled", True)):
            self._imu = CarlaIMU(world, actor, imu_cfg)

        # 6. Odometry (no sensor actor — reads vehicle state directly)
        odom_cfg = cfg.get("odometry", {})
        if bool(odom_cfg.get("enabled", True)):
            self._odometry = CarlaOdometry(odom_cfg)

        # 7. Camera + LiDAR sensors
        cameras_cfg = cfg.get("cameras", [])
        lidars_cfg  = cfg.get("lidars", [])
        tf_cfg = cfg.get("tf", {})
        base_frame_id = str(tf_cfg.get("base_frame_id", odom_cfg.get("child_frame_id", "base_link")))
        odom_frame_id = str(odom_cfg.get("frame_id", "odom"))
        
        # Broadcast static map -> odom transform
        self._ros2_backend.publish_static_transform("map", odom_frame_id, {})
        
        broadcast_sensor_tf = bool(tf_cfg.get("broadcast_sensor_tf", False))
        self._sensor_mgr = CarlaSensorManager(
            world,
            actor,
            cameras_cfg,
            lidars_cfg,
            self._ros2_backend,
            base_frame_id=base_frame_id,
            broadcast_tf=broadcast_sensor_tf,
        )
        self._sensor_mgr.start()

        # Do an initial world tick to let sensors initialize
        self._vehicle.tick()

        # 8. ROS 2 timer — frequency matches CARLA fixed delta (or GPS if async)
        carla_cfg = cfg.get("carla", {})
        if carla_cfg.get("synchronous_mode", True):
            fixed_delta = float(carla_cfg.get("fixed_delta_seconds", 0.05))
            timer_hz = 1.0 / fixed_delta if fixed_delta > 0 else 20.0
        else:
            timer_hz = float(cfg.get("gps", {}).get("update_rate", 10.0))
            
        self._timer = self.create_timer(1.0 / timer_hz, self._timer_callback, callback_group=self._tick_cb_group)

        # High-rate sensor threads
        if self._gps is not None:
            gps_hz = float(cfg.get("gps", {}).get("update_rate", 10.0))
            self._prev_gps_time = time.monotonic()
            self._start_sensor_thread("gps", gps_hz, self._gps_callback)

        if self._battery is not None:
            bat_hz = float(cfg.get("battery", {}).get("update_rate", 10.0))
            self._start_sensor_thread("battery", bat_hz, self._battery_callback)

        if self._imu is not None:
            imu_hz = float(cfg.get("imu", {}).get("update_rate", 50.0))
            self._start_sensor_thread("imu", imu_hz, self._imu_callback)

        if self._odometry is not None:
            odom_hz = float(cfg.get("odometry", {}).get("update_rate", 20.0))
            self._start_sensor_thread("odometry", odom_hz, self._odom_callback)

        self.get_logger().info(
            f"CarlaTelemetryNode started. Timer at {timer_hz:.1f} Hz. "
            f"{len(self._sensor_mgr.cameras)} camera(s), {len(self._sensor_mgr.lidars)} lidar(s). "
            f"Odometry: {'enabled' if self._odometry is not None else 'disabled'}."
        )

    # ------------------------------------------------------------------
    # Timer callback (runs on the ROS 2 executor thread)
    # ------------------------------------------------------------------

    def _timer_callback(self):
        t_total = perf.tick()
        now = time.monotonic()
        dt  = now - self._prev_tick_time
        self._prev_tick_time = now

        # Respect sim start/stop state
        if not self._ros2_backend._sim_running:
            return

        # Advance CARLA one tick (sync mode)
        t_tick = perf.tick()
        self._vehicle.tick()
        perf.record("node.world_tick", t_tick)

        # ── Apply vehicle control from /sim/control/* commands ──────────
        self._ros2_backend.apply_vehicle_control()

        velocity = self._vehicle.get_velocity()
        speed    = self._vehicle.get_speed_ms()

        # ── Speed feedback ────────────────────────────────────────
        self._ros2_backend.publish_speed(speed)
        self._ros2_backend.publish_steering_angles()   # echo last command

        perf.record("node.timer_callback", t_total)

    # ------------------------------------------------------------------
    # Sensor callbacks (run on dedicated python threads)
    # ------------------------------------------------------------------

    def _start_sensor_thread(self, name, hz, callback):
        def loop():
            period = 1.0 / hz
            next_time = time.monotonic()
            while rclpy.ok() and not self._shutdown_event.is_set():
                now = time.monotonic()
                sleep_dur = next_time - now
                if sleep_dur > 0:
                    time.sleep(sleep_dur)
                next_time += period
                if next_time < time.monotonic():
                    next_time = time.monotonic() + period
                
                if self._ros2_backend._sim_running:
                    try:
                        callback()
                    except Exception as exc:
                        self.get_logger().warning(f"Error in {name} thread: {exc}")

        t = threading.Thread(target=loop, name=f"carla_{name}_thread", daemon=True)
        t.start()
        self._sensor_threads.append(t)

    def _gps_callback(self):
        if self._gps is None:
            return
        now = time.monotonic()
        dt = now - self._prev_gps_time
        self._prev_gps_time = now
        velocity = self._vehicle.get_velocity()
        gps_data = self._gps.update(velocity, dt)
        if gps_data is not None:
            self._ros2_backend.publish_gps(gps_data)

    def _battery_callback(self):
        if self._battery is None:
            return
        now = time.monotonic()
        dt = now - self._prev_bat_time
        self._prev_bat_time = now
        speed = self._vehicle.get_speed_ms()
        bat_data = self._battery.update(speed, dt)
        if bat_data is not None:
            self._ros2_backend.publish_battery(bat_data)

    def _imu_callback(self):
        if self._imu is None:
            return
        imu_data = self._imu.get_state()
        if imu_data is not None:
            self._ros2_backend.publish_imu(imu_data)

    def _odom_callback(self):
        if self._odometry is None:
            return
        odom_data = self._odometry.get_state(self._vehicle.actor)
        if odom_data is not None:
            self._ros2_backend.publish_odometry(odom_data)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _teardown(self):
        self.get_logger().info("Shutting down CarlaTelemetryNode …")
        self._shutdown_event.set()

        if self._sensor_mgr is not None:
            try:
                self._sensor_mgr.destroy()
            except Exception as exc:
                self.get_logger().warning(f"Sensor manager destroy error: {exc}")

        for sensor in (self._gps, self._imu):
            if sensor is not None and hasattr(sensor, "destroy"):
                try:
                    sensor.destroy()
                except Exception as exc:
                    self.get_logger().warning(f"Sensor destroy error: {exc}")

        self._gps = None
        self._imu = None
        self._odometry = None
        self._battery = None

        if self._vehicle is not None:
            try:
                self._vehicle.destroy()
            except Exception as exc:
                self.get_logger().warning(f"Vehicle destroy error: {exc}")

        if self._ros2_backend is not None:
            try:
                self._ros2_backend.shutdown()
            except Exception:
                pass

        self.get_logger().info("CarlaTelemetryNode shutdown complete.")

    def destroy_node(self):
        """Called by rclpy on SIGINT/SIGTERM."""
        self._teardown()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    rclpy.init(args=args)

    node = None
    try:
        node = CarlaTelemetryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logging.error("Fatal error: %s", exc, exc_info=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
