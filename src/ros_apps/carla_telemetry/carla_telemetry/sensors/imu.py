"""
| File: imu.py
| Description: CARLA IMU sensor wrapper.
               Wraps sensor.other.imu and collects accelerometer, gyroscope,
               and compass readings for publishing as sensor_msgs/Imu.

CARLA IMU data members:
  data.accelerometer : carla.Vector3D  (m/s², CARLA frame: X=fwd, Y=right, Z=up)
  data.gyroscope     : carla.Vector3D  (rad/s)
  data.compass       : float           (degrees, 0=North, clockwise positive)
  data.transform     : carla.Transform (sensor world pose)
"""
__all__ = ["CarlaIMU"]

import math
import threading
import weakref


class CarlaIMU:
    """CARLA IMU sensor wrapper.

    Output state dict keys:
    ===================== =====================================================
    Key                   Value
    ===================== =====================================================
    ``accel_x/y/z``       Linear acceleration in CARLA body frame (m/s²)
    ``gyro_x/y/z``        Angular velocity (rad/s)
    ``compass``           Compass heading (degrees, 0=North CW)
    ``frame_id``          ROS 2 frame ID string
    ===================== =====================================================
    """

    def __init__(self, carla_world, vehicle_actor, config: dict):
        """
        Args:
            carla_world:   carla.World instance.
            vehicle_actor: Ego vehicle actor.
            config:        IMU config dict from carla_interface_config.yaml.
        """
        self._frame_id = str(config.get("frame_id", "imu_link"))
        self._lock = threading.Lock()
        self._last_data = None

        # Spawn blueprint
        import carla
        bp = carla_world.get_blueprint_library().find("sensor.other.imu")

        noise_attrs = [
            "noise_accel_stddev_x", "noise_accel_stddev_y", "noise_accel_stddev_z",
            "noise_gyro_stddev_x",  "noise_gyro_stddev_y",  "noise_gyro_stddev_z",
            "noise_gyro_bias_x",    "noise_gyro_bias_y",    "noise_gyro_bias_z",
        ]
        for attr in noise_attrs:
            val = config.get(attr)
            if val is not None and bp.has_attribute(attr):
                bp.set_attribute(attr, str(float(val)))

        sp_cfg = config.get("spawn_point", {})
        transform = carla.Transform(
            carla.Location(
                x=float(sp_cfg.get("x", 0.0)),
                y=float(sp_cfg.get("y", 0.0)),
                z=float(sp_cfg.get("z", 0.0)),
            ),
            carla.Rotation(
                roll=float(sp_cfg.get("roll", 0.0)),
                pitch=float(sp_cfg.get("pitch", 0.0)),
                yaw=float(sp_cfg.get("yaw", 0.0)),
            ),
        )
        self._sensor = carla_world.spawn_actor(bp, transform, attach_to=vehicle_actor)

        weak_self = weakref.ref(self)
        self._sensor.listen(lambda data: CarlaIMU._on_imu(weak_self, data))

    # ------------------------------------------------------------------
    # CARLA callback
    # ------------------------------------------------------------------

    @staticmethod
    def _on_imu(weak_self, data):
        self = weak_self()
        if self is None:
            return
        with self._lock:
            self._last_data = data

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_state(self):
        """Return latest IMU state dict or None if no data yet."""
        with self._lock:
            data = self._last_data

        if data is None:
            return None

        limits = (-99.9, 99.9)

        def clamp(v):
            return max(limits[0], min(limits[1], v))

        return {
            "accel_x": clamp(data.accelerometer.x),
            "accel_y": clamp(data.accelerometer.y),
            "accel_z": clamp(data.accelerometer.z),
            "gyro_x":  clamp(data.gyroscope.x),
            "gyro_y":  clamp(data.gyroscope.y),
            "gyro_z":  clamp(data.gyroscope.z),
            "compass": math.degrees(data.compass) if hasattr(data.compass, '__float__') else float(data.compass),
            "frame_id": self._frame_id,
        }

    def destroy(self):
        """Stop and destroy the CARLA IMU sensor."""
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
            except Exception:
                pass
            self._sensor = None
