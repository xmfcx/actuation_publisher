import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from autoware_vehicle_msgs.msg import GearCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationCommand


class ActuationPublisher(Node):
    def __init__(self):
        super().__init__('actuation_publisher')

        # Publishers
        self.publisher_ = self.create_publisher(
            ActuationCommandStamped,
            '/control/command/actuation_cmd',
            10
        )

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.gear_pub = self.create_publisher(
            GearCommand,
            '/control/command/gear_cmd',
            transient_qos
        )

        # Timers
        self.timer_main = self.create_timer(1.0 / 30.0, self.timer_callback_main)
        self.timer_aux = self.create_timer(1.0 / 10.0, self.timer_callback_aux)

    def timer_callback_main(self):
        now = self.get_clock().now().to_msg()

        # ActuationCommandStamped
        msg = ActuationCommandStamped()
        msg.header = Header()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"

        actuation = ActuationCommand()
        actuation.accel_cmd = 0.5
        actuation.brake_cmd = 0.0
        actuation.steer_cmd = 0.0
        msg.actuation = actuation

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published ActuationCommandStamped: {msg}')

    def timer_callback_aux(self):
        now = self.get_clock().now().to_msg()

        # GearCommand
        gear_msg = GearCommand()
        gear_msg.stamp = now
        gear_msg.command = GearCommand.DRIVE
        self.gear_pub.publish(gear_msg)
        self.get_logger().info(f'Published GearCommand {gear_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = ActuationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
