"""
| File: lidar.py
| Description: CARLA LiDAR sensor wrapper — supports both rotary and solid-state.

               lidar_type: "rotary"
                 → sensor.lidar.ray_cast
                 Spinning LiDAR.  Returns XYZI (x, y, z, intensity) point cloud.
                 Supports full noise/dropoff parameter set.

               lidar_type: "solid_state"
                 → sensor.lidar.ray_cast_semantic
                 Non-rotating flash LiDAR.  Returns XYZ + semantic + instance labels.
                 Uses horizontal_fov / vertical_fov.  Dropoff params not applicable.

Data format from CARLA (raw_data byte buffer, dtype float32):
  rotary   : (N, 4)  [x, y, z, intensity]
  solid-state: (N, 6)  [x, y, z, cos_angle, object_idx, semantic_tag]

Both are published as sensor_msgs/PointCloud2 with XYZ fields only (plus intensity
for rotary when available).  The sensor_manager can be extended to publish semantic
fields separately if needed.

Architecture (same backpressure pattern as CarlaCamera):
  CARLA callback → single-slot queue → dedicated publish thread
"""
__all__ = ["CarlaLidar"]

import queue
import struct
import threading
import time
import weakref
import logging

import numpy as np

from carla_telemetry.perf_monitor import perf

logger = logging.getLogger(__name__)


# CARLA LiDAR blueprint IDs
_ROTARY_BLUEPRINT = "sensor.lidar.ray_cast"
_SOLID_BLUEPRINT = "sensor.lidar.ray_cast_semantic"

# Point-cloud fields per row for each type
_ROTARY_FIELDS = 4   # x, y, z, intensity
_SOLID_FIELDS = 6    # x, y, z, cos_angle, object_idx, semantic_tag


class CarlaLidar:
    """CARLA LiDAR sensor wrapper (rotary or solid-state).

    Output data dict (passed to CarlaROS2Backend):
    ============ ============================================================
    Key          Value
    ============ ============================================================
    ``points``   (N, 3) float32 numpy array of XYZ points (CARLA sensor frame)
    ``intensity``(N,)   float32 array (rotary only; zeros for solid-state)
    ``frame_id`` ROS 2 frame ID string
    ============ ============================================================
    """

    # Mapping from lidar_type string to CARLA blueprint
    _BLUEPRINT_MAP = {
        "rotary":      _ROTARY_BLUEPRINT,
        "solid_state": _SOLID_BLUEPRINT,
    }

    def __init__(self, carla_world, vehicle_actor, config: dict):
        """
        Args:
            carla_world:   carla.World instance.
            vehicle_actor: Ego vehicle actor.
            config:        Single LiDAR config dict from carla_interface_config.yaml.
        """
        self._name = str(config.get("name", "lidar"))
        self._frame_id = str(config.get("frame_id", self._name))
        self._lidar_type = str(config.get("lidar_type", "rotary")).lower()
        self._update_rate = float(config.get("update_rate", 20.0))
        self._update_period = 1.0 / self._update_rate
        self._enabled = bool(config.get("enabled", True))

        self._is_solid_state = (self._lidar_type == "solid_state")
        self._n_fields = _SOLID_FIELDS if self._is_solid_state else _ROTARY_FIELDS

        # Topic name stored for backend registration
        self.topic_point_cloud = str(
            config.get("topic_point_cloud", f"lidar/{self._name}/points")
        )

        self._frame_queue: queue.Queue = queue.Queue(maxsize=5)
        self._ros2_backend = None
        self._running = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._sensor = None
        self._accum_lock = threading.Lock()
        self._accum_data = []
        self._accum_start_time = -1.0
        self._accum_capture_time = None

        # Spawn the CARLA LiDAR actor
        import carla

        blueprint_id = self._BLUEPRINT_MAP.get(self._lidar_type, _ROTARY_BLUEPRINT)
        bp = carla_world.get_blueprint_library().find(blueprint_id)

        # ── Common attributes ─────────────────────────────────────────
        self._set_bp_attr(bp, "channels",         config.get("channels", 64))
        self._set_bp_attr(bp, "range",            config.get("range", 85.0))
        self._set_bp_attr(bp, "points_per_second", config.get("points_per_second", 600000))

        if self._is_solid_state:
            # Solid-state: horizontal/vertical FOV instead of rotation_frequency
            self._set_bp_attr(bp, "horizontal_fov",   config.get("horizontal_fov", 120.0))
            self._set_bp_attr(bp, "vertical_fov",     config.get("vertical_fov", 30.0))
            self._rotation_freq = 0.0
        else:
            # Rotary: spinning parameters + noise/dropoff
            self._rotation_freq = float(config.get("rotation_frequency", 20.0))
            self._set_bp_attr(bp, "rotation_frequency", self._rotation_freq)
            self._set_bp_attr(bp, "upper_fov",          config.get("upper_fov", 10.0))
            self._set_bp_attr(bp, "lower_fov",          config.get("lower_fov", -30.0))
            self._set_bp_attr(bp, "atmosphere_attenuation_rate",
                              config.get("atmosphere_attenuation_rate", 0.004))
            self._set_bp_attr(bp, "dropoff_general_rate",
                              config.get("dropoff_general_rate", 0.45))
            self._set_bp_attr(bp, "dropoff_intensity_limit",
                              config.get("dropoff_intensity_limit", 0.8))
            self._set_bp_attr(bp, "dropoff_zero_intensity",
                              config.get("dropoff_zero_intensity", 0.4))

        sp_cfg = config.get("spawn_point", {})
        transform = carla.Transform(
            carla.Location(
                x=float(sp_cfg.get("x", 0.0)),
                y=float(sp_cfg.get("y", 0.0)),
                z=float(sp_cfg.get("z", 2.4)),
            ),
            carla.Rotation(
                roll=float(sp_cfg.get("roll", 0.0)),
                pitch=float(sp_cfg.get("pitch", 0.0)),
                yaw=float(sp_cfg.get("yaw", 0.0)),
            ),
        )
        self._sensor = carla_world.spawn_actor(bp, transform, attach_to=vehicle_actor)

        # Tell CARLA to only fire this sensor at the configured rate
        # (avoids unnecessary callbacks when world tick is faster).
        # Note: sensor_tick is set post-spawn for LiDAR because the rotary
        # accumulation logic relies on receiving sub-sweep measurements.

        # Defer listen() to start() — avoid buffering before thread ready

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _set_bp_attr(bp, key: str, value):
        """Set a blueprint attribute if it exists, ignoring unsupported keys."""
        if bp.has_attribute(key):
            bp.set_attribute(key, str(value))

    # ------------------------------------------------------------------
    # CARLA callback (fires on CARLA worker thread)
    # ------------------------------------------------------------------

    @staticmethod
    def _on_lidar(weak_self, data):
        """Receive raw CARLA LiDAR measurement, accumulate full sweep, and push to queue."""
        import time
        t_cb = perf.tick()
        capture_time = time.time()
        self = weak_self()
        if self is None or not self._running.is_set():
            return
            
        with self._accum_lock:
            if not self._is_solid_state:
                if self._accum_start_time < 0:
                    self._accum_start_time = data.timestamp
                    self._accum_capture_time = capture_time
                    
                self._accum_data.append(data)
                
                rotation_duration = 1.0 / max(self._rotation_freq, 1.0)
                if (data.timestamp - self._accum_start_time) < (rotation_duration * 0.9):
                    return  # keep accumulating
                    
                full_data = list(self._accum_data)
                publish_capture_time = self._accum_capture_time
                self._accum_data.clear()
                self._accum_start_time = -1.0
            else:
                full_data = [data]
                publish_capture_time = capture_time
        
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
                
        try:
            self._frame_queue.put_nowait((full_data, publish_capture_time))
        except queue.Full:
            pass
        perf.record("lidar.callback", t_cb)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self, ros2_backend):
        if self._running.is_set():
            return
        self._ros2_backend = ros2_backend
        self._running.set()

        # Start listening only when publish thread is ready
        weak_self = weakref.ref(self)
        self._sensor.listen(lambda data: CarlaLidar._on_lidar(weak_self, data))
        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            name=f"carla_lidar_{self._name}",
            daemon=True,
        )
        self._publish_thread.start()

    def stop(self):
        self._running.clear()
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=2.0)
            self._publish_thread = None

    def destroy(self):
        self.stop()
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
            except Exception:
                pass
            self._sensor = None

    # ------------------------------------------------------------------
    # Publish loop (dedicated OS thread)
    # ------------------------------------------------------------------

    def _publish_loop(self):
        next_publish_time = time.monotonic()

        while self._running.is_set():
            # ── Sleep until next publish deadline ─────────────────────
            now = time.monotonic()
            sleep_dur = next_publish_time - now
            if sleep_dur > 0:
                time.sleep(sleep_dur)

            # Advance deadline; if we overran, skip missed slots
            next_publish_time += self._update_period
            if next_publish_time < time.monotonic():
                next_publish_time = time.monotonic() + self._update_period

            # ── Drain queue — keep only the newest frame ──────────────
            data_list = None
            capture_time = None
            while not self._frame_queue.empty():
                try:
                    data_list, capture_time = self._frame_queue.get_nowait()
                except queue.Empty:
                    break

            if data_list is None:
                continue

            perf.record_value("lidar.queue_depth", self._frame_queue.qsize())

            accumulated_points = []
            accumulated_intensity = []

            t_parse = perf.tick()
            for data in data_list:
                parsed = self._parse_lidar(data, capture_time)
                if parsed is not None:
                    accumulated_points.append(parsed["points"])
                    accumulated_intensity.append(parsed["intensity"])

            if not accumulated_points:
                continue

            full_points = np.vstack(accumulated_points)
            full_intensity = np.concatenate(accumulated_intensity)
            perf.record("lidar.parse", t_parse)

            publish_capture_time = capture_time

            try:
                if self._ros2_backend is not None:
                    t_pub = perf.tick()
                    self._ros2_backend.publish_point_cloud(
                        full_points,
                        frame_id=self._frame_id,
                        intensity=full_intensity,
                        lidar_name=self._name,
                        capture_time=publish_capture_time,
                    )
                    perf.record("lidar.ros2_publish", t_pub)
            except Exception as exc:
                logger.warning("[CarlaLidar:%s] publish error: %s", self._name, exc)

    def _parse_lidar(self, data, capture_time: float) -> dict | None:
        """Parse raw CARLA LiDAR data into a point cloud dict.

        Returns:
            dict with keys ``points`` (N×3 float32) and ``intensity`` (N float32),
            or ``None`` if no valid points.
        """
        raw = np.frombuffer(data.raw_data, dtype=np.float32)
        if raw.size == 0:
            return None

        n_fields = self._n_fields
        n_points = raw.size // n_fields
        if n_points == 0:
            return None

        pts = raw.reshape((n_points, n_fields))

        # XYZ columns (CARLA sensor frame: X=forward, Y=right, Z=up)
        # CARLA LiDAR uses left-hand coordinate system (Y is mirrored vs ROS REP-103)
        # We flip Y so the point cloud is right-handed for ROS conventions.
        points = np.zeros((n_points, 3), dtype=np.float32)
        points[:, 0] =  pts[:, 0]   # X forward
        points[:, 1] = -pts[:, 1]   # Y left (flip CARLA right-hand)
        points[:, 2] =  pts[:, 2]   # Z up

        if self._is_solid_state:
            intensity = np.zeros(n_points, dtype=np.float32)
        else:
            intensity = pts[:, 3].copy()

        return {"points": points, "intensity": intensity, "capture_time": capture_time}

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def frame_id(self) -> str:
        return self._frame_id

    @property
    def lidar_type(self) -> str:
        return self._lidar_type
