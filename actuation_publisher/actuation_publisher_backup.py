import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationCommand, VehicleEmergencyStamped
from builtin_interfaces.msg import Time
from autoware_vehicle_msgs.msg import GearCommand, ControlModeReport, HazardLightsCommand, TurnIndicatorsCommand
from autoware_control_msgs.msg import Control, Lateral, Longitudinal
from rclpy.qos import QoSProfile, DurabilityPolicy


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

        self.control_mode_pub = self.create_publisher(
            ControlModeReport,
            '/vehicle/status/control_mode',
            10
        )

        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        self.emergency_pub = self.create_publisher(
            VehicleEmergencyStamped,
            '/control/command/emergency_cmd',
            10
        )

        self.hazard_pub = self.create_publisher(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            10
        )

        self.turn_indicators_pub = self.create_publisher(
            TurnIndicatorsCommand,
            '/control/command/turn_indicators_cmd',
            10
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

        # # ControlModeReport
        # control_mode_msg = ControlModeReport()
        # control_mode_msg.stamp = now
        # control_mode_msg.mode = ControlModeReport.AUTONOMOUS
        # self.control_mode_pub.publish(control_mode_msg)
        # self.get_logger().info('Published ControlModeReport (AUTONOMOUS)')
        #
        # # Control
        # control_msg = Control()
        # control_msg.stamp = now
        # control_msg.control_time = now
        #
        # lateral = Lateral()
        # lateral.stamp = now
        # lateral.steering_tire_angle = 0.0
        # control_msg.lateral = lateral
        #
        # longitudinal = Longitudinal()
        # longitudinal.stamp = now
        # longitudinal.velocity = 5.0
        # longitudinal.acceleration = 1.0
        # control_msg.longitudinal = longitudinal
        #
        # self.control_pub.publish(control_msg)
        # self.get_logger().info('Published Control')
        #
        # # Emergency Command
        # emergency_msg = VehicleEmergencyStamped()
        # emergency_msg.stamp = now
        # emergency_msg.emergency = False
        # self.emergency_pub.publish(emergency_msg)
        # self.get_logger().info('Published VehicleEmergencyStamped (emergency=False)')

    def timer_callback_aux(self):
        now = self.get_clock().now().to_msg()

        # GearCommand
        gear_msg = GearCommand()
        gear_msg.stamp = now
        gear_msg.command = GearCommand.DRIVE
        self.gear_pub.publish(gear_msg)
        self.get_logger().info('Published GearCommand (DRIVE)')

        # # HazardLightsCommand
        # hazard_msg = HazardLightsCommand()
        # hazard_msg.stamp = now
        # hazard_msg.command = HazardLightsCommand.DISABLE
        # self.hazard_pub.publish(hazard_msg)
        # self.get_logger().info('Published HazardLightsCommand (DISABLE)')
        #
        # # TurnIndicatorsCommand
        # turn_indicators_msg = TurnIndicatorsCommand()
        # turn_indicators_msg.stamp = now
        # turn_indicators_msg.command = TurnIndicatorsCommand.NO_COMMAND
        # self.turn_indicators_pub.publish(turn_indicators_msg)
        # self.get_logger().info('Published TurnIndicatorsCommand (NO_COMMAND)')


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
