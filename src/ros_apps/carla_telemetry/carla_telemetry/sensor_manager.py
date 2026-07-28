"""
| File: sensor_manager.py
| Description: CarlaSensorManager — spawns, starts, and manages the lifecycle
               of all CARLA sensor actors (cameras and LiDARs).
               Mirrors the role of AsyncSensorManager in Micropolis.Telemetry
               but without asyncio (CARLA sensor data arrives via callbacks on
               CARLA worker threads, so per-sensor OS threads are used instead).

Architecture:
  - For each enabled camera config → spawn CarlaCamera, register publishers, start thread
  - For each enabled lidar config  → spawn CarlaLidar,  register publisher,  start thread
  - IMU is managed separately in the node (it is read on the ROS timer, not a thread)
  - destroy() stops all threads and destroys all CARLA actors cleanly
"""
__all__ = ["CarlaSensorManager"]

import logging

logger = logging.getLogger("carla_telemetry.sensor_manager")


class CarlaSensorManager:
    """Manages multi-camera and multi-LiDAR CARLA sensor actors.

    Args:
        carla_world:   carla.World instance.
        vehicle_actor: Ego vehicle actor (sensors attach to this).
        cameras_config: List of camera config dicts from carla_interface_config.yaml.
        lidars_config:  List of lidar config dicts from carla_interface_config.yaml.
        ros2_backend:   ``CarlaROS2Backend`` instance used for publishing.
    """

    def __init__(
        self,
        carla_world,
        vehicle_actor,
        cameras_config: list,
        lidars_config: list,
        ros2_backend,
        base_frame_id: str = "base_link",
        broadcast_tf: bool = False,
    ):
        from carla_telemetry.sensors.camera import CarlaCamera
        from carla_telemetry.sensors.lidar import CarlaLidar

        self._cameras: list[CarlaCamera] = []
        self._lidars:  list[CarlaLidar]  = []
        self._ros2 = ros2_backend
        self._base_frame_id = str(base_frame_id)
        self._broadcast_tf = bool(broadcast_tf)

        # ── Cameras ───────────────────────────────────────────────────
        for cfg in cameras_config:
            if not bool(cfg.get("enabled", True)):
                logger.info(
                    "[SensorManager] Camera '%s' disabled — skipping.", cfg.get("name", "?")
                )
                continue
            try:
                cam = CarlaCamera(carla_world, vehicle_actor, cfg)
                self._cameras.append(cam)
                # Register publishers before starting the thread
                ros2_backend.register_camera(
                    camera_name=cam.frame_id,
                    topic_rgb=cfg.get("topic_rgb", ""),
                    topic_info=cfg.get("topic_camera_info", ""),
                )
                if self._broadcast_tf:
                    self._ros2.publish_static_transform(
                        parent_frame=self._base_frame_id,
                        child_frame=cam.frame_id,
                        spawn_point=cfg.get("spawn_point", {}),
                    )
                    self._ros2.publish_optical_transform(
                        parent_frame=cam.frame_id,
                        child_frame=cam.frame_id + "_optical",
                    )
                logger.info("[SensorManager] Camera '%s' spawned.", cam.name)
            except Exception as exc:
                logger.error(
                    "[SensorManager] Failed to spawn camera '%s': %s",
                    cfg.get("name", "?"), exc,
                )

        # ── LiDARs ────────────────────────────────────────────────────
        for cfg in lidars_config:
            if not bool(cfg.get("enabled", True)):
                logger.info(
                    "[SensorManager] LiDAR '%s' disabled — skipping.", cfg.get("name", "?")
                )
                continue
            try:
                lidar = CarlaLidar(carla_world, vehicle_actor, cfg)
                self._lidars.append(lidar)
                ros2_backend.register_lidar(
                    lidar_name=lidar.name,
                    topic=cfg.get("topic_point_cloud", ""),
                )
                if self._broadcast_tf:
                    self._ros2.publish_static_transform(
                        parent_frame=self._base_frame_id,
                        child_frame=lidar.frame_id,
                        spawn_point=cfg.get("spawn_point", {}),
                    )
                logger.info(
                    "[SensorManager] LiDAR '%s' (%s) spawned.",
                    lidar.name, lidar.lidar_type,
                )
            except Exception as exc:
                logger.error(
                    "[SensorManager] Failed to spawn lidar '%s': %s",
                    cfg.get("name", "?"), exc,
                )

        logger.info(
            "[SensorManager] Created %d camera(s) and %d lidar(s).",
            len(self._cameras), len(self._lidars),
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Start all publish threads."""
        for cam in self._cameras:
            cam.start(self._ros2)
        for lidar in self._lidars:
            lidar.start(self._ros2)
        logger.info(
            "[SensorManager] Started %d camera thread(s) and %d lidar thread(s).",
            len(self._cameras), len(self._lidars),
        )

    def stop(self):
        """Stop all publish threads (sensors still active in CARLA)."""
        for cam in self._cameras:
            cam.stop()
        for lidar in self._lidars:
            lidar.stop()
        logger.info("[SensorManager] All publish threads stopped.")

    def destroy(self):
        """Stop threads and destroy all CARLA sensor actors."""
        for cam in self._cameras:
            try:
                cam.destroy()
            except Exception as exc:
                logger.warning("[SensorManager] Camera destroy error: %s", exc)
        self._cameras.clear()

        for lidar in self._lidars:
            try:
                lidar.destroy()
            except Exception as exc:
                logger.warning("[SensorManager] LiDAR destroy error: %s", exc)
        self._lidars.clear()

        logger.info("[SensorManager] All sensors destroyed.")

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def cameras(self):
        return list(self._cameras)

    @property
    def lidars(self):
        return list(self._lidars)
