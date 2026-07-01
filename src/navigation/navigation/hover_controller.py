import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

GRAVITY_MS2 = 9.81
DRONE_MASS_KG = 1.5
TARGET_ALTITUDE_M = 3.0


class HoverController(Node):
    """Applies just enough upward force to counteract gravity, keeping the
    drone hovering at a fixed altitude. This is a placeholder controller for
    Phase 1 — later phases replace it with the full navigation stack."""

    def __init__(self):
        super().__init__('hover_controller')
        self.publisher = self.create_publisher(Wrench, 'force', 10)
        self.timer = self.create_timer(0.05, self.publish_hover_force)
        self.get_logger().info('Hover controller started.')

    def publish_hover_force(self):
        wrench = Wrench()
        wrench.force.z = DRONE_MASS_KG * GRAVITY_MS2
        self.publisher.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
