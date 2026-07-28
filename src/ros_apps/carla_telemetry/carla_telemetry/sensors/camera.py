"""
| File: camera.py
| Description: CARLA camera sensor wrapper.
               Spawns a CARLA camera actor (any sensor.camera.* type) attached
               to the ego vehicle, receives frames via listen() callback, and
               exposes the latest frame for ROS 2 publishing.

               Each camera runs a dedicated publish thread that drains a
               single-slot queue (newest-frame backpressure — stale frames
               are dropped, not queued) and calls the ROS 2 backend.

CARLA camera sensor types supported (set via config.type):
  sensor.camera.rgb                  - standard BGRA image
  sensor.camera.depth                - depth map (encoded as RGB, use converter)
  sensor.camera.semantic_segmentation
  sensor.camera.instance_segmentation
  sensor.camera.optical_flow
  sensor.camera.normals

Camera intrinsics are computed analytically from fov and image_size (no USD needed).

Output data dict (passed to ROS2Backend):
  rgb           : (H, W, 3) uint8 numpy array (RGB)
  intrinsics    : (3, 3) float64 camera matrix K
  width/height  : int pixels
  frame_id      : str
  encoding      : str ("rgb8")
"""
__all__ = ["CarlaCamera"]

import math
import queue
import threading
import time
import weakref
import logging

import numpy as np

from carla_telemetry.perf_monitor import perf

logger = logging.getLogger(__name__)


class CarlaCamera:
    """CARLA camera actor wrapper with per-camera publish thread.

    Architecture mirrors Micropolis.Telemetry.sensors.camera.TelemetryCamera:
    - CARLA callback → single-slot queue (newest only, stale dropped)
    - A dedicated publish thread drains the queue at ``update_rate`` Hz
    - Rate gate prevents ROS 2 flooding when CARLA ticks faster than desired
    """

    def __init__(self, carla_world, vehicle_actor, config: dict):
        """
        Args:
            carla_world:   carla.World instance.
            vehicle_actor: Ego vehicle actor to attach camera to.
            config:        Single camera config dict from carla_interface_config.yaml.
        """
        self._name = str(config.get("name", "camera"))
        self._frame_id = str(config.get("frame_id", self._name))
        self._update_rate = float(config.get("update_rate", 30.0))
        self._update_period = 1.0 / self._update_rate
        self._enabled = bool(config.get("enabled", True))

        self._image_size_x = int(config.get("image_size_x", 640))
        self._image_size_y = int(config.get("image_size_y", 480))
        self._fov = float(config.get("fov", 90.0))

        # Topic configuration (stored for backend registration)
        self.topic_rgb = str(config.get("topic_rgb", f"{self._name}/rgb"))
        self.topic_camera_info = str(config.get("topic_camera_info", f"{self._name}/camera_info"))

        # Pre-compute camera intrinsics once (constant for fixed resolution+FOV)
        self._intrinsics = self._compute_intrinsics()

        # Small buffer queue to absorb OS scheduling jitter without dropping frames.
        self._frame_queue: queue.Queue = queue.Queue(maxsize=5)

        self._ros2_backend = None   # injected by CarlaSensorManager after init
        self._running = threading.Event()
        self._publish_thread: threading.Thread | None = None
        self._sensor = None
        self._last_queued_sim_time = -1.0
        self._accum_lock = threading.Lock()

        # Generate camera blueprinta actor
        import carla
        bp = carla_world.get_blueprint_library().find(str(config.get("type", "sensor.camera.rgb")))
        bp.set_attribute("image_size_x", str(self._image_size_x))
        bp.set_attribute("image_size_y", str(self._image_size_y))
        if bp.has_attribute("fov"):
            bp.set_attribute("fov", str(self._fov))

        # Tell CARLA to only render this camera at the configured rate.
        # Without this, the camera renders every world.tick() even if the
        # publish thread only consumes at update_rate — wasting GPU cycles.
        if bp.has_attribute("sensor_tick"):
            sensor_tick = 1.0 / max(self._update_rate, 1.0)
            bp.set_attribute("sensor_tick", str(sensor_tick))

        # Apply any extra blueprint attributes from config
        skip_keys = {"name", "enabled", "type", "spawn_point", "image_size_x",
                     "image_size_y", "fov", "update_rate", "frame_id",
                     "topic_rgb", "topic_camera_info"}
        for key, value in config.items():
            if key not in skip_keys and bp.has_attribute(str(key)):
                bp.set_attribute(str(key), str(value))

        sp_cfg = config.get("spawn_point", {})
        transform = carla.Transform(
            carla.Location(
                x=float(sp_cfg.get("x", 1.5)),
                y=float(sp_cfg.get("y", 0.0)),
                z=float(sp_cfg.get("z", 1.5)),
            ),
            carla.Rotation(
                roll=float(sp_cfg.get("roll", 0.0)),
                pitch=float(sp_cfg.get("pitch", 0.0)),
                yaw=float(sp_cfg.get("yaw", 0.0)),
            ),
        )
        self._sensor = carla_world.spawn_actor(bp, transform, attach_to=vehicle_actor)

        # Defer listen() to start() — avoid buffering frames before thread ready

    # ------------------------------------------------------------------
    # Intrinsics computation
    # ------------------------------------------------------------------

    def _compute_intrinsics(self) -> np.ndarray:
        """Compute 3×3 intrinsic matrix K from FOV and image dimensions.

        CARLA camera model: horizontal FOV is the full angle.
        fx = fy = (image_width / 2) / tan(hfov / 2)
        cx = image_width  / 2
        cy = image_height / 2
        """
        fx = self._image_size_x / (2.0 * math.tan(math.radians(self._fov) / 2.0))
        fy = fx  # square pixels
        cx = self._image_size_x / 2.0
        cy = self._image_size_y / 2.0
        return np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1],
        ], dtype=np.float64)

    # ------------------------------------------------------------------
    # CARLA callback (fires on CARLA worker thread)
    # ------------------------------------------------------------------

    @staticmethod
    def _on_image(weak_self, image):
        """Receive a raw CARLA image and push to the queue with capture time."""
        import time
        t_cb = perf.tick()
        capture_time = time.time()
        self = weak_self()
        if self is None or not self._running.is_set():
            return
            
        with self._accum_lock:
            sim_time = image.timestamp
            if self._last_queued_sim_time >= 0.0 and (sim_time - self._last_queued_sim_time) < (self._update_period * 0.9):
                return
            self._last_queued_sim_time = sim_time
            
        # Drop oldest frame if queue is full (keep newest)
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._frame_queue.put_nowait((image, capture_time))
        except queue.Full:
            pass
        perf.record("cam.callback", t_cb)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self, ros2_backend):
        """Start the publish thread.

        Args:
            ros2_backend: A ``CarlaROS2Backend`` instance.
        """
        if self._running.is_set():
            return
        self._ros2_backend = ros2_backend
        self._running.set()

        # Start listening only when publish thread is ready
        weak_self = weakref.ref(self)
        self._sensor.listen(lambda image: CarlaCamera._on_image(weak_self, image))
        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            name=f"carla_cam_{self._name}",
            daemon=True,
        )
        self._publish_thread.start()

    def stop(self):
        """Stop the publish thread."""
        self._running.clear()
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=2.0)
            self._publish_thread = None

    def destroy(self):
        """Stop thread and destroy the CARLA sensor actor."""
        self.stop()
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
            except Exception:
                pass
            self._sensor = None

    # ------------------------------------------------------------------
    # Publish loop (runs in dedicated OS thread)
    # ------------------------------------------------------------------

    def _publish_loop(self):
        """Drain the frame queue and publish at the configured rate."""
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
            image = None
            capture_time = None
            while not self._frame_queue.empty():
                try:
                    image, capture_time = self._frame_queue.get_nowait()
                except queue.Empty:
                    break

            if image is None:
                continue

            perf.record_value("cam.queue_depth", self._frame_queue.qsize())

            try:
                t_parse = perf.tick()
                data = self._parse_image(image, capture_time)
                perf.record("cam.parse", t_parse)

                if data is not None and self._ros2_backend is not None:
                    t_pub = perf.tick()
                    self._ros2_backend.publish_camera_image(data)
                    self._ros2_backend.publish_camera_info(data)
                    perf.record("cam.ros2_publish", t_pub)
            except Exception as exc:
                logger.warning("[CarlaCamera:%s] publish error: %s", self._name, exc)

    def _parse_image(self, image, capture_time: float) -> dict | None:
        """Convert a carla.Image to a data dict compatible with CarlaROS2Backend."""
        # CARLA images are BGRA (4 channels, uint8)
        raw = np.frombuffer(image.raw_data, dtype=np.uint8)
        if raw.size == 0:
            return None

        img = raw.reshape((image.height, image.width, 4))  # BGRA
        # Convert BGRA → RGB
        rgb = img[:, :, :3][:, :, ::-1].copy()   # BGR → RGB

        return {
            "rgb":          rgb,
            "intrinsics":   self._intrinsics,
            "width":        image.width,
            "height":       image.height,
            "frame_id":     self._frame_id,
            "encoding":     "rgb8",
            "capture_time": capture_time,
        }

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def frame_id(self) -> str:
        return self._frame_id
