import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sim_manager_msgs.msg import TireForces


class FrictionDecay(Node):
    """Ramps /sim/control/tire_friction down and prints the effective value.

    The bridge owns the physics write: a second client calling
    apply_physics_control on the same vehicle stalls the server.
    """

    def __init__(self, start=1.5, target=1.0, step=0.01, interval=10.0):
        super().__init__('friction_decay')
        self.friction = start
        self.target = target
        self.step = step
        self.effective = None
        self.pub = self.create_publisher(Float32, '/sim/control/tire_friction', 10)
        self.create_subscription(TireForces, '/sim/feedback/tire_forces',
                                 self._on_forces, 10)
        self.create_timer(interval, self._tick)

    def _on_forces(self, msg):
        self.effective = [round(v, 3) for v in msg.tire_friction]

    def _tick(self):
        self.pub.publish(Float32(data=float(self.friction)))
        self.get_logger().info(
            f"commanded {self.friction:.3f} -> effective FL/FR/RL/RR: "
            f"{self.effective}")
        if self.friction <= self.target:
            self.get_logger().info("Friction decay complete!")
            raise SystemExit
        self.friction -= self.step


def main():
    rclpy.init()
    node = FrictionDecay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
