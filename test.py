import rclpy
from rclpy.node import Node
from sim_manager_msgs.msg import TireForces
from sim_manager_msgs.srv import SetTireFriction


class FrictionDecay(Node):
    """Ramps the tire friction down and prints the effective value.

    The bridge owns the physics write: a second client touching the same
    vehicle stalls the server.
    """

    def __init__(self):
        super().__init__('friction_decay')
        self.effective = None
        self.client = self.create_client(SetTireFriction,
                                         '/sim/control/set_tire_friction')
        self.create_subscription(TireForces, '/sim/feedback/tire_forces',
                                 self._on_forces, 10)

    def _on_forces(self, msg):
        self.effective = [round(v, 3) for v in msg.tire_friction]

    def set_friction(self, friction):
        future = self.client.call_async(
            SetTireFriction.Request(friction=float(friction)))
        # The response is the back-pressure: one call in flight keeps the ramp
        # from outrunning the simulator.
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def ramp(node, start=1.5, target=1.0, step=0.01):
    friction = start
    while friction >= target:
        response = node.set_friction(friction)
        node.get_logger().info(
            f"commanded {friction:.3f} -> {response.message} "
            f"effective FL/FR/RL/RR: {node.effective}")
        if not response.success:
            return
        friction -= step
    node.get_logger().info("Friction decay complete!")


def main():
    rclpy.init()
    node = FrictionDecay()
    try:
        node.client.wait_for_service()
        ramp(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
