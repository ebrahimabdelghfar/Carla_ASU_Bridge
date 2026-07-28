"""
| File: odometry.py
| Description: CarlaOdometry — computes and publishes nav_msgs/Odometry from CARLA
               vehicle state (position, orientation, linear + angular velocity).

               Odometry is ground-truth from CARLA (perfect pose integration) —
               no wheel-encoder noise model.  The node can layer wheel-slip noise
               on top if desired in the future.

Coordinate convention:
  CARLA uses a LEFT-HAND system:
    X = forward, Y = right, Z = up
    Rotation is in DEGREES; positive yaw is CLOCKWISE looking down.

  ROS REP-103 (right-hand):
    X = forward, Y = left, Z = up
    Positive yaw is COUNTER-CLOCKWISE.

  Conversions applied here:
    pos_y_ros   = -pos_y_carla       (right → left)
    vel_y_ros   = -vel_y_carla
    ang_vel_ros = radians; negate Y and Z components
    pitch/yaw   = negated (axes flip)

Topic: nav_msgs/Odometry (frame_id: "odom", child_frame_id: "base_link")

Optional TF broadcast is supported when the `broadcast_tf` config flag is set.
"""
__all__ = ["CarlaOdometry"]

import math
import time


class CarlaOdometry:
    """Computes nav_msgs/Odometry from CARLA vehicle state.

    Args:
        config: ``odometry`` section of carla_interface_config.yaml.
    """

    def __init__(self, config: dict = None):
        if config is None:
            config = {}
        self._frame_id       = str(config.get("frame_id", "odom"))
        self._child_frame_id = str(config.get("child_frame_id", "base_link"))
        self._update_rate    = float(config.get("update_rate", 20.0))
        self._broadcast_tf   = bool(config.get("broadcast_tf", False))
        self._tf_broadcaster = None   # set by node if broadcast_tf is True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_state(self, vehicle) -> dict | None:
        """Compute odometry state dict from a CARLA vehicle actor.

        Args:
            vehicle: ``carla.Actor`` (ego vehicle).

        Returns:
            State dict ready for ``CarlaROS2Backend.publish_odometry()``,
            or ``None`` if rate limit not reached.
        """
        transform   = vehicle.get_transform()
        velocity    = vehicle.get_velocity()        # m/s, CARLA frame
        ang_vel_raw = vehicle.get_angular_velocity()  # deg/s, CARLA frame

        # ── Position (CARLA→ROS) ─────────────────────────────────────
        loc = transform.location
        pos_x =  float(loc.x)
        pos_y = -float(loc.y)   # flip Y (right→left)
        pos_z =  float(loc.z)

        # ── Orientation (Euler deg → quaternion, ROS convention) ─────
        rot   = transform.rotation
        roll  = math.radians(float(rot.roll))
        pitch = math.radians(-float(rot.pitch))    # negate: CARLA Y flipped
        yaw   = math.radians(-float(rot.yaw))      # negate: CW→CCW

        qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)

        # ── Linear velocity (CARLA→ROS) ──────────────────────────────
        vx =  float(velocity.x)
        vy = -float(velocity.y)   # flip
        vz =  float(velocity.z)

        # ── Angular velocity (deg/s → rad/s, CARLA→ROS) ─────────────
        wx =  math.radians(float(ang_vel_raw.x))
        wy = -math.radians(float(ang_vel_raw.y))   # negate Y
        wz = -math.radians(float(ang_vel_raw.z))   # negate Z (CW→CCW)

        return {
            "frame_id":       self._frame_id,
            "child_frame_id": self._child_frame_id,
            "pos_x": pos_x, "pos_y": pos_y, "pos_z": pos_z,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "vx": vx, "vy": vy, "vz": vz,
            "wx": wx, "wy": wy, "wz": wz,
            "broadcast_tf": self._broadcast_tf,
        }

    @property
    def broadcast_tf(self) -> bool:
        return self._broadcast_tf


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert RPY (radians) to quaternion (x, y, z, w).
    Intrinsic ZYX convention used by CARLA / ROS.
    """
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
