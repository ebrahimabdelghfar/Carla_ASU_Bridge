"""
| File: ros2_backend.py
| Description: CarlaROS2Backend — ROS 2 publisher/service layer for CARLA telemetry.
               Mirrors Micropolis.Telemetry.backends.ros2_backend.ROS2Backend
               with an identical public API so sensors and the node can call
               publish_gps(), publish_battery(), publish_point_cloud(), etc.
               unchanged.

Topics published (all under /<namespace>/):
========================== ========================
Topic                      Message type
========================== ========================
feedback/gps               NavSatFix
feedback/gps_vel           TwistStamped
feedback/battery/state     BatteryState
feedback/imu               sensor_msgs/Imu
<camera>/rgb               Image  (per camera)
<camera>/camera_info       CameraInfo  (per camera)
<lidar>/points             PointCloud2  (per lidar)
========================== ========================

Services:
control/battery/start_drain   Trigger
control/battery/stop_drain    Trigger
control/battery/start_charge  Trigger
control/battery/stop_charge   Trigger
"""
__all__ = ["CarlaROS2Backend"]

import logging
import math
import carla

logger = logging.getLogger("carla_telemetry.ros2_backend")


class CarlaROS2Backend:
    """ROS 2 publisher + service layer for CARLA telemetry.

    Args:
        node:             A ``rclpy.node.Node`` instance (shared with the main node).
        config:           ``ros2`` section of carla_interface_config.yaml.
        battery_controller: A ``CarlaBattery`` instance for service callbacks.
        vehicle:          The ``CarlaVehicle`` instance for control callbacks.
                          If None, control subscriptions are still created but
                          commands are buffered until ``set_vehicle()`` is called.
    """

    def __init__(self, node, config: dict = None, battery_controller=None, vehicle=None):
        if config is None:
            config = {}
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_srvs.srv import Trigger, SetBool
        from std_msgs.msg import Header, Float32, Bool, Empty
        from sensor_msgs.msg import (
            NavSatFix, NavSatStatus, BatteryState,
            PointCloud2, PointField, Image, CameraInfo, Imu, JointState,
        )
        from geometry_msgs.msg import TwistStamped, Pose2D
        from nav_msgs.msg import Odometry

        self._node = node
        self._rclpy = rclpy
        self._NavSatFix = NavSatFix
        self._NavSatStatus = NavSatStatus
        self._BatteryState = BatteryState
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._TwistStamped = TwistStamped
        self._Header = Header
        self._Trigger = Trigger
        self._SetBool = SetBool
        self._Image = Image
        self._CameraInfo = CameraInfo
        self._Imu = Imu
        self._Odometry = Odometry
        self._JointState = JointState
        self._Float32 = Float32
        self._tf_broadcaster = None   # created on demand if broadcast_tf requested
        self._static_tf_broadcaster = None
        self._static_transforms = []

        # ── Vehicle reference for control callbacks ────────────────────
        self._vehicle = vehicle          # carla_telemetry.vehicle.CarlaVehicle
        self._max_rpm = 150.0            # overridden via set_control_config()
        self._max_steer_deg = 16.0

        # Latest commanded state (applied each world tick)
        self._cmd_velocity_rpm   = 0.0
        self._cmd_steering_deg   = 0.0
        self._cmd_brake          = False
        self._last_steering_deg  = 0.0   # for steering_angle echo topic

        self._battery_controller = battery_controller
        self._namespace = str(config.get("namespace", "sim")).strip("/")
        self._topics_cfg = dict(config.get("topics", {}))
        self._services_cfg = dict(config.get("services", {}))

        # Per-camera publishers: {camera_name: {"rgb": Pub, "info": Pub}}
        self._camera_pubs: dict = {}
        # Per-lidar publishers: {lidar_name: Pub}
        self._lidar_pubs: dict = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Core feedback publishers ──────────────────────────────────
        self._gps_pub = node.create_publisher(
            NavSatFix,
            self._topic(self._topics_cfg.get("feedback_gps", "feedback/gps")),
            qos,
        )
        self._gps_vel_pub = node.create_publisher(
            TwistStamped,
            self._topic(self._topics_cfg.get("feedback_gps_vel", "feedback/gps_vel")),
            qos,
        )
        self._battery_pub = node.create_publisher(
            BatteryState,
            self._topic(self._topics_cfg.get("feedback_battery_state", "feedback/battery/state")),
            qos,
        )
        self._imu_pub = node.create_publisher(
            Imu,
            self._topic(self._topics_cfg.get("feedback_imu", "feedback/imu")),
            qos,
        )
        self._odom_pub = node.create_publisher(
            Odometry,
            # ICD: /sim/odom (not under feedback/)
            self._topic(self._topics_cfg.get("odom", "odom")),
            qos,
        )
        # Speed feedback: /sim/feedback/speed
        self._speed_pub = node.create_publisher(
            Float32,
            self._topic(self._topics_cfg.get("feedback_speed", "feedback/speed")),
            qos,
        )
        # Steering angle echo: /sim/feedback/steering_angle
        self._steer_echo_pub = node.create_publisher(
            Float32,
            self._topic(self._topics_cfg.get("feedback_steering_angle", "feedback/steering_angle")),
            qos,
        )
        # 4-wheel steering angles: /sim/feedback/steering_angles
        self._steer_angles_pub = node.create_publisher(
            JointState,
            self._topic(self._topics_cfg.get("feedback_steering_angles", "feedback/steering_angles")),
            qos,
        )

        # ── Control subscriptions ─────────────────────────────────────
        self._vel_rpm_sub = node.create_subscription(
            Float32,
            self._topic(self._topics_cfg.get("control_velocity_rpm", "control/velocity_rpm")),
            self._on_velocity_rpm,
            qos,
        )
        self._steer_sub = node.create_subscription(
            Float32,
            self._topic(self._topics_cfg.get("control_steering_angle_deg", "control/steering_angle_deg")),
            self._on_steering_angle,
            qos,
        )
        self._brake_sub = node.create_subscription(
            Bool,
            self._topic(self._topics_cfg.get("control_brake", "control/brake")),
            self._on_brake,
            qos,
        )
        self._spawn_sub = node.create_subscription(
            Pose2D,
            "/sim/spawn_point",   # ICD: absolute topic, not under namespace
            self._on_spawn_point,
            qos,
        )

        # ── Simulator lifecycle services (/micropolis/..., absolute) ──
        self._sim_start_srv = node.create_service(
            Trigger, "/micropolis/sim/start", self._on_sim_start
        )
        self._sim_stop_srv = node.create_service(
            Trigger, "/micropolis/sim/stop", self._on_sim_stop
        )
        # Legacy Empty topics for backward compatibility
        self._sim_start_topic_sub = node.create_subscription(
            Empty, "/micropolis/sim/start",
            lambda _: self._sim_state_request("start"), qos
        )
        self._sim_stop_topic_sub = node.create_subscription(
            Empty, "/micropolis/sim/stop",
            lambda _: self._sim_state_request("stop"), qos
        )
        self._sim_running = True   # CARLA starts in running state

        # ── Battery services ──────────────────────────────────────────
        svc = self._services_cfg
        self._start_drain_srv = node.create_service(
            Trigger, self._topic(svc.get("start_drain", "control/battery/start_drain")),
            self._on_start_drain,
        )
        self._stop_drain_srv = node.create_service(
            Trigger, self._topic(svc.get("stop_drain", "control/battery/stop_drain")),
            self._on_stop_drain,
        )
        self._start_charge_srv = node.create_service(
            Trigger, self._topic(svc.get("start_charge", "control/battery/start_charge")),
            self._on_start_charge,
        )
        self._stop_charge_srv = node.create_service(
            Trigger, self._topic(svc.get("stop_charge", "control/battery/stop_charge")),
            self._on_stop_charge,
        )

        # ── Light services ────────────────────────────────────────────
        self._high_beams_srv = node.create_service(
            SetBool, self._topic(svc.get("high_beams", "control/lights/high_beams")),
            self._on_high_beams,
        )
        self._low_beams_srv = node.create_service(
            SetBool, self._topic(svc.get("low_beams", "control/lights/low_beams")),
            self._on_low_beams,
        )
        self._left_blinker_srv = node.create_service(
            SetBool, self._topic(svc.get("left_blinker", "control/lights/left_blinker")),
            self._on_left_blinker,
        )
        self._right_blinker_srv = node.create_service(
            SetBool, self._topic(svc.get("right_blinker", "control/lights/right_blinker")),
            self._on_right_blinker,
        )
        self._siren_srv = node.create_service(
            SetBool, self._topic(svc.get("siren", "control/lights/siren")),
            self._on_siren,
        )

        logger.info(
            "[CarlaROS2Backend] Initialized. GPS → %s | Odom → %s | Control → %s",
            self._topic(self._topics_cfg.get("feedback_gps", "feedback/gps")),
            self._topic(self._topics_cfg.get("odom", "odom")),
            self._topic(self._topics_cfg.get("control_velocity_rpm", "control/velocity_rpm")),
        )

    # ------------------------------------------------------------------
    # Topic helpers
    # ------------------------------------------------------------------

    def _topic(self, suffix: str) -> str:
        """Build absolute topic path from namespace + suffix."""
        suffix = str(suffix).strip()
        if not suffix:
            return "/"
        if suffix.startswith("/"):
            return suffix
        ns = self._namespace
        if not ns:
            return f"/{suffix}"
        return f"/{ns}/{suffix}"

    # ------------------------------------------------------------------
    # Camera publisher registration
    # ------------------------------------------------------------------

    def register_camera(self, camera_name: str, topic_rgb: str = "", topic_info: str = ""):
        """Create ROS 2 publishers for one camera.

        Args:
            camera_name: Unique camera identifier (frame_id).
            topic_rgb:   Relative/absolute topic for Image. Defaults to ``<camera_name>/rgb``.
            topic_info:  Relative/absolute topic for CameraInfo. Defaults to ``<camera_name>/camera_info``.
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        if camera_name in self._camera_pubs:
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        rgb_topic = self._topic(topic_rgb or f"{camera_name}/rgb")
        info_topic = self._topic(topic_info or f"{camera_name}/camera_info")

        self._camera_pubs[camera_name] = {
            "rgb":  self._node.create_publisher(self._Image, rgb_topic, qos),
            "info": self._node.create_publisher(self._CameraInfo, info_topic, qos),
        }
        logger.info("[CarlaROS2Backend] Camera '%s' → %s | %s", camera_name, rgb_topic, info_topic)

    # ------------------------------------------------------------------
    # LiDAR publisher registration
    # ------------------------------------------------------------------

    def register_lidar(self, lidar_name: str, topic: str = ""):
        """Create a ROS 2 PointCloud2 publisher for one LiDAR.

        Args:
            lidar_name: Unique LiDAR identifier.
            topic:      Relative/absolute topic. Defaults to ``lidar/<lidar_name>/points``.
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        if lidar_name in self._lidar_pubs:
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pc_topic = self._topic(topic or f"lidar/{lidar_name}/points")
        self._lidar_pubs[lidar_name] = self._node.create_publisher(
            self._PointCloud2, pc_topic, qos
        )
        logger.info("[CarlaROS2Backend] LiDAR '%s' → %s", lidar_name, pc_topic)

    # ------------------------------------------------------------------
    # Battery controller
    # ------------------------------------------------------------------

    def set_battery_controller(self, battery_controller):
        self._battery_controller = battery_controller

    def set_vehicle(self, vehicle):
        """Inject the CarlaVehicle reference after construction."""
        self._vehicle = vehicle

    def set_control_config(self, max_rpm: float = 150.0, max_steer_deg: float = 70.0):
        """Configure the scale factors for RPM→throttle and steer conversion.

        Args:
            max_rpm:       RPM value that maps to full throttle (1.0).
            max_steer_deg: Steering angle (degrees) that maps to full steer (1.0).
        """
        self._max_rpm = max_rpm
        self._max_steer_deg = max_steer_deg

    def _battery_available(self) -> bool:
        if self._battery_controller is None:
            logger.warning("[CarlaROS2Backend] Battery command ignored: no controller.")
            return False
        return True

    def _ok_response(self, response, message: str):
        response.success = True
        response.message = message
        return response

    def _on_start_drain(self, request, response):
        if self._battery_available():
            self._battery_controller.start_draining()
        return self._ok_response(response, "Battery draining started.")

    def _on_stop_drain(self, request, response):
        if self._battery_available():
            self._battery_controller.stop_draining()
        return self._ok_response(response, "Battery draining stopped.")

    def _on_start_charge(self, request, response):
        if self._battery_available():
            self._battery_controller.start_charging()
        return self._ok_response(response, "Battery charging started.")

    def _on_stop_charge(self, request, response):
        if self._battery_available():
            self._battery_controller.stop_charging()
        return self._ok_response(response, "Battery charging stopped.")

    # ── Light Services Callbacks ───────────────────────────────────

    def _set_light_state(self, flag, enable: bool):
        if not self._vehicle:
            return False
        import carla
        lights = self._vehicle.actor.get_light_state()
        if enable:
            lights |= flag
        else:
            lights &= ~flag
        self._vehicle.actor.set_light_state(carla.VehicleLightState(lights))
        return True

    def _on_high_beams(self, request, response):
        import carla
        success = self._set_light_state(carla.VehicleLightState.HighBeam, request.data)
        response.success = success
        response.message = "High beams toggled." if success else "No vehicle."
        return response

    def _on_low_beams(self, request, response):
        import carla
        success = self._set_light_state(carla.VehicleLightState.Position, request.data)
        success = self._set_light_state(carla.VehicleLightState.LowBeam, request.data)
        response.success = success
        response.message = "Low beams toggled." if success else "No vehicle."
        return response

    def _on_left_blinker(self, request, response):
        import carla
        success = self._set_light_state(carla.VehicleLightState.LeftBlinker, request.data)
        response.success = success
        response.message = "Left blinker toggled." if success else "No vehicle."
        return response

    def _on_right_blinker(self, request, response):
        import carla
        success = self._set_light_state(carla.VehicleLightState.RightBlinker, request.data)
        response.success = success
        response.message = "Right blinker toggled." if success else "No vehicle."
        return response

    def _on_siren(self, request, response):
        import carla
        success = self._set_light_state(carla.VehicleLightState.Special1, request.data)
        response.success = success
        response.message = "Siren toggled." if success else "No vehicle."
        return response

    # ------------------------------------------------------------------
    # Control subscribers (ICD: /sim/control/*)
    # ------------------------------------------------------------------

    def _on_velocity_rpm(self, msg):
        """Receive wheel velocity command in RPM.

        ICD: positive = forward, negative = reverse.
        Converts to CARLA throttle [0, 1] and reverse flag.
        """
        self._cmd_velocity_rpm = float(msg.data)

    def _on_steering_angle(self, msg):
        """Receive steering angle in degrees.

        ICD convention: +left / −right.
        CARLA VehicleControl.steer: −10=right, +1.0=left.
        We flip sign to match CARLA.
        """
        self._cmd_steering_deg = float(msg.data)

    def _on_brake(self, msg):
        """Receive emergency brake flag.

        ICD: true = full brake; zeroes throttle and steer.
        """
        self._cmd_brake = bool(msg.data)

    def _on_spawn_point(self, msg):
        """Teleport the ego vehicle to the requested (x, y, theta) pose.

        ICD: geometry_msgs/Pose2D — x, y in CARLA world metres, theta in radians.
        """
        if self._vehicle is None:
            logger.warning("[CarlaROS2Backend] spawn_point received but no vehicle set.")
            return
        transform = self._vehicle.actor.get_transform()
        transform.location.x = float(msg.x)
        transform.location.y = -float(msg.y)   # ROS→CARLA Y flip
        transform.rotation.yaw = -math.degrees(float(msg.theta))  # CCW→CW
        self._vehicle.actor.set_transform(transform)
        logger.info(
            "[CarlaROS2Backend] Teleported ego to (%.1f, %.1f, yaw=%.1f°).",
            msg.x, msg.y, math.degrees(msg.theta)
        )

    def apply_vehicle_control(self):
        """Convert latest control commands into a CARLA VehicleControl and apply it.

        Must be called once per world tick (from the node timer callback).
        Safe to call even when self._vehicle is None (no-op).

        Signal conversions (ICD):
          velocity_rpm → throttle [0, 1] + reverse bool
          steering_angle_deg → steer [−1, 1]   (+= left in ICD = − in CARLA)
          brake = True → throttle=0, steer=0, hand_brake=False, brake=1.0
        """
        if self._vehicle is None:
            return

        import carla as _carla_mod

        rpm     = self._cmd_velocity_rpm
        steer_d = self._cmd_steering_deg
        braking = self._cmd_brake

        ctrl = _carla_mod.VehicleControl()

        if braking:
            # Full emergency brake — ICD: zeroes throttle and steer
            ctrl.throttle = 0.0
            ctrl.steer    = 0.0
            ctrl.brake    = 1.0
            ctrl.hand_brake = False
            ctrl.reverse  = False
        else:
            ctrl.brake = 0.0
            ctrl.hand_brake = False

            if rpm >= 0.0:
                ctrl.throttle = min(1.0, abs(rpm) / max(self._max_rpm, 1.0))
                ctrl.reverse  = False
            else:
                ctrl.throttle = min(1.0, abs(rpm) / max(self._max_rpm, 1.0))
                ctrl.reverse  = True

            # ICD: +steer_deg = left; CARLA steer: +1 = right → flip sign
            ctrl.steer = max(-1.0, min(1.0, -steer_d / max(self._max_steer_deg, 1.0)))

        self._vehicle.actor.apply_control(ctrl)
        self._last_steering_deg = steer_d if not braking else 0.0

    # ------------------------------------------------------------------
    # Speed + steering echo publishers (ICD)
    # ------------------------------------------------------------------

    def publish_speed(self, speed_ms: float):
        """Publish /sim/feedback/speed (Float32, m/s)."""
        msg = self._Float32()
        msg.data = float(speed_ms)
        self._speed_pub.publish(msg)

    def publish_steering_angles(self, steer_deg: float | None = None):
        """Publish /sim/feedback/steering_angle (echo) and /sim/feedback/steering_angles (JointState).

        Args:
            steer_deg: Current steering angle in degrees. Uses last commanded value when None.
        """
        if steer_deg is None:
            if self._vehicle and self._vehicle.actor:
                # Read true physical wheel angle from CARLA (FL wheel) in degrees
                # CARLA: +right, -left. ICD: +left, -right.
                carla_steer_deg = self._vehicle.actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
                steer_deg = -carla_steer_deg
            else:
                steer_deg = self._last_steering_deg

        # Scalar echo
        echo = self._Float32()
        echo.data = float(steer_deg)
        self._steer_echo_pub.publish(echo)

        # JointState for all 4 wheels: FL, FR, RL, RR
        # ICD: position in degrees, velocity in rad/s.
        # Front wheels steer, rear wheels fixed at 0.
        steer_rad = math.radians(steer_deg)
        js = self._JointState()
        js.header.stamp = self._node.get_clock().now().to_msg()
        js.name     = ["wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"]
        js.position = [steer_deg, steer_deg, 0.0, 0.0]   # degrees per ICD
        js.velocity = [steer_rad, steer_rad, 0.0, 0.0]   # rad/s per ICD
        js.effort   = [0.0, 0.0, 0.0, 0.0]
        self._steer_angles_pub.publish(js)

    # ------------------------------------------------------------------
    # Simulator lifecycle service callbacks
    # ------------------------------------------------------------------

    def _on_sim_start(self, request, response):
        self._sim_state_request("start")
        return self._ok_response(response, "Start request queued.")

    def _on_sim_stop(self, request, response):
        self._sim_state_request("stop")
        return self._ok_response(response, "Stop request queued.")

    def _sim_state_request(self, state: str):
        """Handle sim start/stop request.

        In CARLA synchronous mode the world tick is driven by the node timer.
        Pausing is achieved by stopping the ticking loop via the vehicle.
        """
        if state == "start":
            self._sim_running = True
            if self._vehicle is not None:
                try:
                    self._vehicle.resume()
                except AttributeError:
                    pass   # CarlaVehicle.resume() is optional
            logger.info("[CarlaROS2Backend] Simulation START requested.")
        else:
            self._sim_running = False
            if self._vehicle is not None:
                try:
                    self._vehicle.pause()
                except AttributeError:
                    pass
            logger.info("[CarlaROS2Backend] Simulation STOP requested.")

    # ------------------------------------------------------------------
    # GPS publish
    # ------------------------------------------------------------------

    def publish_gps(self, gps_data: dict):
        """Publish NavSatFix + TwistStamped from a GPS state dict."""
        stamp = self._node.get_clock().now().to_msg()

        fix = self._NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps_link"

        status = self._NavSatStatus()
        status.status = self._NavSatStatus.STATUS_FIX
        status.service = self._NavSatStatus.SERVICE_GPS
        fix.status = status

        fix.latitude  = gps_data["latitude"]
        fix.longitude = gps_data["longitude"]
        fix.altitude  = gps_data["altitude"]

        eph = gps_data.get("eph", 1.0)
        epv = gps_data.get("epv", 1.0)
        fix.position_covariance = [
            eph**2, 0.0, 0.0,
            0.0, eph**2, 0.0,
            0.0, 0.0, epv**2,
        ]
        fix.position_covariance_type = self._NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(fix)

        twist = self._TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "map"
        twist.twist.linear.x = gps_data["velocity_north"]
        twist.twist.linear.y = gps_data["velocity_east"]
        twist.twist.linear.z = gps_data["velocity_down"]
        self._gps_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Battery publish
    # ------------------------------------------------------------------

    def publish_battery(self, battery_data: dict):
        """Publish BatteryState from a battery state dict."""
        msg = self._BatteryState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.voltage  = float(battery_data["voltage"])
        msg.current  = -float(battery_data["current"])   # convention: negative = discharging
        msg.charge   = float(battery_data["charge"])
        msg.capacity = float(battery_data["charge"])
        msg.design_capacity = float(battery_data["charge"])
        msg.percentage = float(battery_data["charge_fraction"])
        msg.power_supply_status = (
            self._BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if not battery_data["is_depleted"]
            else self._BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        )
        msg.power_supply_health = self._BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = self._BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self._battery_pub.publish(msg)

    # ------------------------------------------------------------------
    # IMU publish
    # ------------------------------------------------------------------

    def publish_imu(self, imu_data: dict):
        """Publish sensor_msgs/Imu from a CarlaIMU state dict.

        CARLA body frame (X=forward, Y=right, Z=up) → ROS REP-103 (X=forward, Y=left, Z=up):
        accel_x → +x, accel_y → -y (flip), accel_z → +z
        gyro: same flip on y
        """
        msg = self._Imu()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = imu_data.get("frame_id", "imu_link")

        # Linear acceleration (m/s²) — CARLA Y is rightward (flip for ROS)
        msg.linear_acceleration.x =  imu_data["accel_x"]
        msg.linear_acceleration.y = -imu_data["accel_y"]
        msg.linear_acceleration.z =  imu_data["accel_z"]
        msg.linear_acceleration_covariance = [0.0] * 9   # unknown

        # Angular velocity (rad/s)
        msg.angular_velocity.x =  imu_data["gyro_x"]
        msg.angular_velocity.y = -imu_data["gyro_y"]
        msg.angular_velocity.z =  imu_data["gyro_z"]
        msg.angular_velocity_covariance = [0.0] * 9

        compass_rad = math.radians(imu_data.get("compass", 0.0))
        yaw = math.pi / 2.0 - compass_rad  # ROS: CCW positive, compass: CW positive, offset by 90 deg for East
        half = yaw / 2.0
        msg.orientation.w = math.cos(half)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(half)
        msg.orientation_covariance = [0.0] * 9

        self._imu_pub.publish(msg)

    # ------------------------------------------------------------------
    # Camera publish (called from CarlaCamera publish thread)
    # ------------------------------------------------------------------

    def publish_camera_image(self, camera_data: dict):
        """Publish sensor_msgs/Image for one camera frame.

        Args:
            camera_data: Dict with keys ``rgb``, ``width``, ``height``, ``frame_id``.
        """
        import numpy as np

        frame_id = camera_data.get("frame_id", "camera")
        if frame_id not in self._camera_pubs:
            self.register_camera(frame_id)

        rgb = camera_data["rgb"]
        height, width = rgb.shape[:2]
        
        capture_time = camera_data.get("capture_time", None)
        if capture_time is not None:
            from builtin_interfaces.msg import Time
            sec = int(capture_time)
            nanosec = int((capture_time - sec) * 1e9)
            stamp = Time(sec=sec, nanosec=nanosec)
        else:
            stamp = self._node.get_clock().now().to_msg()

        msg = self._Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width  = width
        msg.encoding = camera_data.get("encoding", "rgb8")
        msg.is_bigendian = 0
        msg.step = width * 3
        import array
        msg.data = array.array('B', np.ascontiguousarray(rgb).tobytes())
        self._camera_pubs[frame_id]["rgb"].publish(msg)

    def publish_camera_info(self, camera_data: dict):
        """Publish sensor_msgs/CameraInfo for one camera frame.

        Args:
            camera_data: Dict with keys ``intrinsics``, ``width``, ``height``, ``frame_id``.
        """
        import numpy as np

        frame_id = camera_data.get("frame_id", "camera")
        if frame_id not in self._camera_pubs:
            self.register_camera(frame_id)

        K      = camera_data["intrinsics"]   # (3, 3)
        width  = camera_data["width"]
        height = camera_data["height"]
        
        capture_time = camera_data.get("capture_time", None)
        if capture_time is not None:
            from builtin_interfaces.msg import Time
            sec = int(capture_time)
            nanosec = int((capture_time - sec) * 1e9)
            stamp = Time(sec=sec, nanosec=nanosec)
        else:
            stamp = self._node.get_clock().now().to_msg()

        msg = self._CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width  = width
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = K.flatten().tolist()[:9]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = K
        msg.p = P.flatten().tolist()[:12]

        self._camera_pubs[frame_id]["info"].publish(msg)

    # ------------------------------------------------------------------
    # Odometry publish
    # ------------------------------------------------------------------

    def publish_odometry(self, odom_data: dict):
        """Publish nav_msgs/Odometry and optionally broadcast a TF transform.

        Args:
            odom_data: Dict from ``CarlaOdometry.get_state()``.
        """
        stamp = self._node.get_clock().now().to_msg()

        msg = self._Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_data["frame_id"]
        msg.child_frame_id  = odom_data["child_frame_id"]

        # Pose
        msg.pose.pose.position.x = odom_data["pos_x"]
        msg.pose.pose.position.y = odom_data["pos_y"]
        msg.pose.pose.position.z = odom_data["pos_z"]
        msg.pose.pose.orientation.x = odom_data["qx"]
        msg.pose.pose.orientation.y = odom_data["qy"]
        msg.pose.pose.orientation.z = odom_data["qz"]
        msg.pose.pose.orientation.w = odom_data["qw"]
        # Diagonal covariance (unknown — set small constants)
        msg.pose.covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
        ]

        # Twist (body-frame velocities)
        msg.twist.twist.linear.x  = odom_data["vx"]
        msg.twist.twist.linear.y  = odom_data["vy"]
        msg.twist.twist.linear.z  = odom_data["vz"]
        msg.twist.twist.angular.x = odom_data["wx"]
        msg.twist.twist.angular.y = odom_data["wy"]
        msg.twist.twist.angular.z = odom_data["wz"]
        msg.twist.covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
        ]
        self._odom_pub.publish(msg)

        # Optional TF broadcast: odom → base_link
        if odom_data.get("broadcast_tf", False):
            self._broadcast_tf_transform(stamp, odom_data)

    def _broadcast_tf_transform(self, stamp, odom_data: dict):
        """Broadcast odom → base_link TF2 transform."""
        try:
            from tf2_ros import TransformBroadcaster
            from geometry_msgs.msg import TransformStamped
        except ImportError:
            logger.warning(
                "[CarlaROS2Backend] tf2_ros not available; TF broadcast skipped."
            )
            return

        if self._tf_broadcaster is None:
            self._tf_broadcaster = TransformBroadcaster(self._node)

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = odom_data["frame_id"]
        t.child_frame_id  = odom_data["child_frame_id"]
        t.transform.translation.x = odom_data["pos_x"]
        t.transform.translation.y = odom_data["pos_y"]
        t.transform.translation.z = odom_data["pos_z"]
        t.transform.rotation.x = odom_data["qx"]
        t.transform.rotation.y = odom_data["qy"]
        t.transform.rotation.z = odom_data["qz"]
        t.transform.rotation.w = odom_data["qw"]
        self._tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Static TF publish (sensor frames)
    # ------------------------------------------------------------------

    def publish_static_transform(self, parent_frame: str, child_frame: str, spawn_point: dict):
        """Broadcast a static TF from base_link to a sensor frame.

        Args:
            parent_frame: Parent frame ID (typically base_link).
            child_frame:  Sensor frame ID (camera or lidar frame).
            spawn_point:  Dict with x, y, z, roll, pitch, yaw (CARLA frame, degrees).
        """
        if not parent_frame or not child_frame:
            return

        try:
            from tf2_ros import StaticTransformBroadcaster
            from geometry_msgs.msg import TransformStamped
        except ImportError:
            logger.warning(
                "[CarlaROS2Backend] tf2_ros not available; static TF skipped for %s.",
                child_frame,
            )
            return

        if self._static_tf_broadcaster is None:
            self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

        sp = spawn_point or {}
        x = float(sp.get("x", 0.0))
        y = float(sp.get("y", 0.0))
        z = float(sp.get("z", 0.0))
        roll = float(sp.get("roll", 0.0))
        pitch = float(sp.get("pitch", 0.0))
        yaw = float(sp.get("yaw", 0.0))

        qx, qy, qz, qw = self._carla_rpy_to_ros_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = -y   # CARLA right → ROS left
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._static_transforms.append(t)
        self._static_tf_broadcaster.sendTransform(self._static_transforms)

    def publish_optical_transform(self, parent_frame: str, child_frame: str):
        """Broadcast a static TF from a standard ROS camera frame to an optical frame.
        Standard: X=forward, Y=left, Z=up
        Optical:  Z=forward, X=right, Y=down
        """
        try:
            from geometry_msgs.msg import TransformStamped
            import math
        except ImportError:
            return

        if self._static_tf_broadcaster is None:
            return

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # Quaternion for -90 yaw, 0 pitch, -90 roll
        # qx=-0.5, qy=0.5, qz=-0.5, qw=0.5
        t.transform.rotation.x = -0.5
        t.transform.rotation.y =  0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w =  0.5
        self._static_transforms.append(t)
        self._static_tf_broadcaster.sendTransform(self._static_transforms)

    # ------------------------------------------------------------------
    # PointCloud2 publish (called from CarlaLidar publish thread)
    # ------------------------------------------------------------------

    def publish_point_cloud(self, points, frame_id: str, intensity=None, lidar_name: str = "", capture_time: float | None = None):
        """Publish sensor_msgs/PointCloud2.

        Args:
            points:      (N, 3) float32 numpy array (XYZ).
            frame_id:    ROS 2 frame ID for the header.
            intensity:   Optional (N,) float32 array. Included as 4th field if provided.
            lidar_name:  Key for per-lidar publisher lookup.
            capture_time: Optional simulation time override.
        """
        import numpy as np
        import struct

        if points is None or len(points) == 0:
            return

        points = np.ascontiguousarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            return

        # Determine which publisher to use
        pub_key = lidar_name or frame_id
        if pub_key not in self._lidar_pubs:
            self.register_lidar(pub_key, f"lidar/{pub_key}/points")

        if capture_time is not None:
            from builtin_interfaces.msg import Time
            sec = int(capture_time)
            nanosec = int((capture_time - sec) * 1e9)
            stamp = Time(sec=sec, nanosec=nanosec)
        else:
            stamp = self._node.get_clock().now().to_msg()

        # Build PointCloud2 manually (no sensor_msgs_py dependency required)
        has_intensity = intensity is not None and len(intensity) == len(points)
        point_step = 16 if has_intensity else 12   # 4 bytes per float × 3 or 4

        fields = [
            self._PointField(name="x", offset=0,  datatype=self._PointField.FLOAT32, count=1),
            self._PointField(name="y", offset=4,  datatype=self._PointField.FLOAT32, count=1),
            self._PointField(name="z", offset=8,  datatype=self._PointField.FLOAT32, count=1),
        ]
        if has_intensity:
            fields.append(
                self._PointField(name="intensity", offset=12, datatype=self._PointField.FLOAT32, count=1)
            )

        n_points = len(points)
        if has_intensity:
            intensity = np.ascontiguousarray(intensity, dtype=np.float32)
            data = np.zeros((n_points, 4), dtype=np.float32)
            data[:, :3] = points
            data[:, 3]  = intensity
        else:
            data = points

        msg = self._PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = str(frame_id)
        msg.height = 1
        msg.width  = n_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points
        import array
        msg.data = array.array('B', data.tobytes())
        msg.is_dense = True

        self._lidar_pubs[pub_key].publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _carla_rpy_to_ros_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
        """Convert CARLA RPY degrees (left-hand) to ROS quaternion.

        CARLA: X forward, Y right, Z up, degrees, CW yaw positive.
        ROS:   X forward, Y left,  Z up, radians, CCW yaw positive.
        """
        roll = math.radians(roll_deg)
        pitch = math.radians(-pitch_deg)
        yaw = math.radians(-yaw_deg)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    # ------------------------------------------------------------------
    # Node spin
    # ------------------------------------------------------------------

    def spin_once(self):
        """Process pending ROS 2 callbacks (non-blocking)."""
        # Spinning is handled by the rclpy executor in the main node;
        # this is a no-op kept for API parity with Micropolis backend.
        pass

    def shutdown(self):
        """Destroy publishers. Node lifecycle managed externally."""
        logger.info("[CarlaROS2Backend] Shutdown complete.")
