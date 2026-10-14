"""Drive a gentle weave and change tire stiffness and friction under the car.

Fails if the speed collapses while the tires change, the way it used to when
every runtime physics write stopped the car (bug-037), if a stiffness write
undoes the commanded friction, or if the bridge does not read back what was
commanded. Needs the bridge running with control.source: ackermann_drive.
Respawns the ego at the configured spawn pose first, so runs never drive into
the barrier.
"""
import json
import math
import statistics
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sim_manager_msgs.msg import TireForces
from sim_manager_msgs.srv import SetTireFriction, SetTireStiffness
from std_msgs.msg import String

BRIDGE = '/ASU_RT_Carla_Telemetry_Node'
# The run covers about 50 m; spawn has ~70 m of clear asphalt before a crest
# that throws the car whether or not the tires are touched.
CRUISE = 2.0  # m/s
# Cosine weave: the heading swings around the spawn heading with zero mean,
# so the car keeps to its lane.
STEER_AMPLITUDE = 0.1  # rad
STEER_PERIOD = 4.0  # s
# A write costs about 1% of speed for one frame; below this is a stall.
MIN_FRACTION = 0.9


class StiffnessProbe(Node):

    def __init__(self):
        super().__init__('stiffness_probe')
        self.speeds = []
        self.physics = None
        self.friction = None
        self.start = time.monotonic()
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.stiffness = self.create_client(SetTireStiffness,
                                            '/sim/control/set_tire_stiffness')
        self.grip = self.create_client(SetTireFriction,
                                       '/sim/control/set_tire_friction')
        # /odom runs above the 30 Hz world tick, so a one-tick dip shows.
        self.create_subscription(
            Odometry, '/odom',
            lambda m: self.speeds.append(
                math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)),
            qos_profile_sensor_data)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sim/feedback/vehicle_physics',
                                 self._on_physics, latched)
        self.create_subscription(TireForces, '/sim/feedback/tire_forces',
                                 self._on_forces, 10)
        self.create_timer(0.05, self._command)

    def _on_physics(self, msg):
        self.physics = json.loads(msg.data)

    def _on_forces(self, msg):
        self.friction = statistics.fmean(msg.tire_friction)

    def _command(self):
        cmd = AckermannDriveStamped()
        cmd.drive.speed = CRUISE
        t = time.monotonic() - self.start
        cmd.drive.steering_angle = STEER_AMPLITUDE * math.cos(
            2 * math.pi * t / STEER_PERIOD)
        self.drive.publish(cmd)

    def wheel(self):
        return self.physics['wheels'][0] if self.physics else {}

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, client, request):
        client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def respawn(self):
        """Lifecycle-reset the bridge: configure spawns a fresh ego."""
        change = self.create_client(ChangeState, f'{BRIDGE}/change_state')
        get = self.create_client(GetState, f'{BRIDGE}/get_state')
        for t in (Transition.TRANSITION_DEACTIVATE,
                  Transition.TRANSITION_CLEANUP,
                  Transition.TRANSITION_CONFIGURE):
            self.call(change, ChangeState.Request(
                transition=Transition(id=t)))
        # auto_start activates on configure; activate by hand if it did not.
        self.spin_for(1.0)
        if self.call(get, GetState.Request()).current_state.id != \
                State.PRIMARY_STATE_ACTIVE:
            self.call(change, ChangeState.Request(
                transition=Transition(id=Transition.TRANSITION_ACTIVATE)))
        self.physics = None
        while self.physics is None:
            self.spin_for(0.1)
        self.start = time.monotonic()

    def set_stiffness(self, lat=0.0, lon=0.0):
        resp = self.call(self.stiffness, SetTireStiffness.Request(
            lat_stiff_value=float(lat), long_stiff_value=float(lon)))
        assert resp.success, resp.message

    def set_friction(self, friction):
        resp = self.call(self.grip,
                         SetTireFriction.Request(friction=float(friction)))
        assert resp.success, resp.message

    def phase(self, name, action):
        self.speeds.clear()
        action()
        s = self.speeds or [float('nan')]
        self.get_logger().info(
            f'{name:<30} n={len(s):4d} min={min(s):.2f} '
            f'mean={statistics.fmean(s):.2f} max={max(s):.2f} | '
            f'lat={self.wheel().get("lat_stiff_value")} '
            f'long={self.wheel().get("long_stiff_value")} '
            f'mu={self.friction:.3f}')
        return min(s)


def ramp(node, start, stop, steps, apply):
    def run():
        for i in range(steps + 1):
            apply(start + (stop - start) * i / steps)
        node.spin_for(1.0)
    return run


def main():
    rclpy.init()
    node = StiffnessProbe()
    try:
        node.respawn()
        node.spin_for(8.0)  # accelerate and settle
        lat0 = node.wheel()['lat_stiff_value']
        lon0 = node.wheel()['long_stiff_value']
        node.set_friction(1.0)
        node.spin_for(1.0)
        mu_full = node.friction

        def friction_and_stiffness(x):
            node.set_friction(1.0 - 0.4 * x)
            node.set_stiffness(lat=lat0 * (1.0 - 0.5 * x))

        mins = [
            node.phase('baseline', lambda: node.spin_for(2.0)),
            node.phase('single lat step',
                       lambda: (node.set_stiffness(lat=lat0 * 0.5),
                                node.spin_for(2.0))),
            node.phase('lat ramp x0.5 -> x2',
                       ramp(node, lat0 * 0.5, lat0 * 2, 20,
                            lambda v: node.set_stiffness(lat=v))),
            node.phase('long ramp x1 -> x0.3',
                       ramp(node, lon0, lon0 * 0.3, 20,
                            lambda v: node.set_stiffness(lon=v))),
            node.phase('friction 1->0.6 + lat x1->x0.5',
                       ramp(node, 0.0, 1.0, 20, friction_and_stiffness)),
        ]
        assert abs(node.wheel()['lat_stiff_value'] - lat0 * 0.5) < 1e-3
        assert abs(node.wheel()['long_stiff_value'] - lon0 * 0.3) < 1e-3
        # Stiffness writes rebuild the PhysX vehicle; the friction must
        # survive them.
        ratio = node.friction / mu_full
        assert abs(ratio - 0.6) < 0.02, f'friction ratio {ratio:.3f} != 0.6'
        floor = MIN_FRACTION * CRUISE
        assert min(mins) >= floor, f'speed fell to {min(mins):.2f} < {floor}'
        node.get_logger().info(f'PASS: speed never below {floor:.2f} m/s')
        node.set_stiffness(lat0, lon0)
        node.set_friction(1.0)
    finally:
        node.drive.publish(AckermannDriveStamped())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
