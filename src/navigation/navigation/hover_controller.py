import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from ros_gz_interfaces.msg import Entity
from sensor_msgs.msg import NavSatFix

TARGET_ENTITY = Entity(name='drone_1::base_link', type=Entity.LINK)

# gz-sim's ApplyLinkWrench system adds each message on the "persistent"
# wrench topic to whatever standing force is already applied to the entity,
# rather than replacing it — so republishing the same absolute force every
# control tick makes the applied force grow without bound. Publishing the
# delta from the last commanded force instead makes the *cumulative* total
# on the Gazebo side track the intended absolute value.

GRAVITY_MS2 = 9.81
DRONE_MASS_KG = 1.5
TARGET_ALTITUDE_M = 3.0
CONTROL_PERIOD_S = 0.05
HOVER_FORCE_N = DRONE_MASS_KG * GRAVITY_MS2

# Roughly critically damped around a 1.5 rad/s natural frequency.
KP = 4.0
KI = 0.2
KD = 4.0

INTEGRAL_LIMIT = 10.0
MAX_FORCE_N = 3.0 * HOVER_FORCE_N


class HoverController(Node):
    """Feed-forward gravity compensation plus a PID loop on altitude error
    (fed by the GPS sensor bridged from Gazebo) that drives the drone to
    TARGET_ALTITUDE_M. This is a placeholder controller for Phase 1 — later
    phases replace it with the full navigation stack."""

    def __init__(self):
        super().__init__('hover_controller')
        self.publisher = self.create_publisher(EntityWrench, 'force', 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps', self.on_gps, 10)
        self.timer = self.create_timer(CONTROL_PERIOD_S, self.publish_force)
        self.force_z = HOVER_FORCE_N
        self.applied_force_z = 0.0
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_gps_time = None
        self.get_logger().info('Hover controller started.')

    def on_gps(self, msg):
        # PID terms are updated here, on arrival of a new altitude reading,
        # rather than on the faster publish timer — the GPS sensor's update
        # rate is the real sample rate of this control loop, and computing a
        # derivative against the timer period instead of the actual time
        # since the last reading exaggerates it whenever the two rates drift
        # out of sync, which destabilizes the loop.
        now = self.get_clock().now()
        error = TARGET_ALTITUDE_M - msg.altitude

        if self.previous_gps_time is not None:
            dt = (now - self.previous_gps_time).nanoseconds / 1e9
            self.integral_error += error * dt
            self.integral_error = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral_error))
            derivative_error = (error - self.previous_error) / dt
            force_z = (
                HOVER_FORCE_N
                + KP * error + KI * self.integral_error + KD * derivative_error)
            self.force_z = max(-MAX_FORCE_N, min(MAX_FORCE_N, force_z))

        self.previous_error = error
        self.previous_gps_time = now

    def publish_force(self):
        delta = self.force_z - self.applied_force_z
        self.applied_force_z = self.force_z

        msg = EntityWrench()
        msg.entity = TARGET_ENTITY
        msg.wrench.force.z = delta
        self.publisher.publish(msg)


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
