import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from autoware_vehicle_msgs.msg import GearCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationCommand
import math


class ActuationPublisher(Node):
    def __init__(self):
        super().__init__('actuation_publisher')

        # --- Configuration: Define your sequence of unified control waves here ---
        # The signal oscillates between -1.0 (max brake) and +1.0 (max accel).
        self.wave_queue = [
            # stop - start
            {
                'offset': -1.0,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 1.0
            },
            {
                'offset': 0.0,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 1.0
            },


            # 0.1 - 1
            {
                'offset': 0.5,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 4.0
            },
            {
                'offset': 0.1,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 35.0
            },

            # 0.1 - 2
            {
                'offset': 0.5,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 4.0
            },
            {
                'offset': 0.1,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 35.0
            },

            # 0.1 - 3
            {
                'offset': 0.5,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 4.0
            },
            {
                'offset': 0.1,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 30.0
            },

            # 0.1 - 4
            {
                'offset': 1.0,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 4.0
            },
            {
                'offset': 0.1,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 30.0
            },

            # stop
            {
                'offset': -1.0,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 1.0
            },
            {
                'offset': 0.0,
                'amplitude': 0.0,
                'frequency': 0.05,
                'duration': 1.0
            },
            # {
            #     'offset': 0.5,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 16.0
            # },
            # {
            #     'offset': -0.7,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 6.0
            # },
            # {
            #     'offset': 0.5,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 18.0
            # },
            # {
            #     'offset': -0.7,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 6.0
            # },

            # {
            #     'offset': 0.4,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 35.0
            # },
            # {
            #     'offset': -0.6,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 6.0
            # },
            # {
            #     'offset': 0.4,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 35.0
            # },
            # {
            #     'offset': -0.6,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 6.0
            # },

            # {
            #     'offset': 0.3,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 40.0
            # },
            # {
            #     'offset': 1.0,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 3.0
            # },
            # {
            #     'offset': 0.3,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 60.0
            # },
            # {
            #     'offset': -0.5,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 5.0
            # },
            # {
            #     'offset': 0.3,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 10.0
            # },
            # {
            #     'offset': -0.5,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 3.0
            # },

            # # 0.2 - 1
            # {
            #     'offset': 0.2,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 36.0
            # },
            #
            # # 0.2 - 2
            # {
            #     'offset': 1.0,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 4.0
            # },
            # {
            #     'offset': 0.2,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 34.0
            # },
            #
            # # 0.2 - 3
            # {
            #     'offset': 1.0,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 4.0
            # },
            # {
            #     'offset': 0.2,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 29.0
            # },
            #
            # # 0.2 - 4
            # {
            #     'offset': 1.0,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 4.0
            # },
            # {
            #     'offset': 0.2,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 20.0
            # },
            #
            # # 0.2 - 5
            # {
            #     'offset': 1.0,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 4.0
            # },
            # {
            #     'offset': 0.2,
            #     'amplitude': 0.0,
            #     'frequency': 0.05,
            #     'duration': 20.0
            # },
        ]
        # --- End of Configuration ---

        # State variables for managing the wave queue
        self.current_wave_index = 0
        self.wave_start_time = self.get_clock().now()

        # Publishers
        self.actuation_pub = self.create_publisher(
            ActuationCommandStamped,
            '/control/command/actuation_cmd',
            10  # QoS depth
        )

        # Use TRANSIENT_LOCAL to ensure the gear command is latched
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.gear_pub = self.create_publisher(
            GearCommand,
            '/control/command/gear_cmd',
            transient_qos
        )

        # Set gear to DRIVE once at startup
        self.set_gear()

        # Timer to publish actuation commands at 30 Hz
        self.actuation_timer = self.create_timer(1.0 / 30.0, self.publish_actuation_command)
        self.get_logger().info("Actuation publisher node started with unified control.")
        if self.wave_queue:
            self.get_logger().info(f"Starting with wave #1")

    def set_gear(self):
        """Publishes the DRIVE gear command once."""
        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.command = GearCommand.DRIVE
        self.gear_pub.publish(gear_msg)
        self.get_logger().info('Published GearCommand: DRIVE (this will be latched)')

    def publish_actuation_command(self):
        """Calculates and publishes the actuation command at 30 Hz."""
        now = self.get_clock().now()
        control_signal = 0.0

        # Check if there are still waves left in the queue
        if self.current_wave_index < len(self.wave_queue):
            current_wave_params = self.wave_queue[self.current_wave_index]
            duration_of_current_wave = Duration(seconds=current_wave_params['duration'])

            # Check if the current wave's time has elapsed
            if now - self.wave_start_time >= duration_of_current_wave:
                self.wave_start_time = now
                self.current_wave_index += 1

                if self.current_wave_index < len(self.wave_queue):
                    self.get_logger().info(f"Switching to wave #{self.current_wave_index + 1}")
                else:
                    self.get_logger().info("Wave sequence finished. Control signal will be zero.")

            # If the current index is still valid, calculate the sine value
            if self.current_wave_index < len(self.wave_queue):
                p = self.wave_queue[self.current_wave_index]
                time_into_wave = (now - self.wave_start_time).nanoseconds / 1e9

                # Sinusoidal calculation: y(t) = offset + A * sin(2 * pi * f * t)
                calculated_value = p['offset'] + p['amplitude'] * math.sin(
                    2 * math.pi * p['frequency'] * time_into_wave
                )

                # Clamp the signal to the [-1.0, 1.0] range
                control_signal = max(-1.0, min(1.0, calculated_value))

        # Map the single control_signal to accel and brake commands
        accel_cmd = 0.0
        brake_cmd = 0.0
        if control_signal > 0:
            accel_cmd = control_signal
        elif control_signal < 0:
            brake_cmd = -control_signal  # Brake command is positive

        # Construct and publish the actuation message
        msg = ActuationCommandStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"

        actuation = ActuationCommand()
        actuation.accel_cmd = accel_cmd
        actuation.brake_cmd = brake_cmd
        actuation.steer_cmd = 0.0
        msg.actuation = actuation

        self.actuation_pub.publish(msg)
        self.get_logger().info(
            f'Signal: {control_signal:+.3f} -> Accel: {accel_cmd:.3f}, Brake: {brake_cmd:.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActuationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the vehicle by sending a brake command before exiting
        final_msg = ActuationCommandStamped()
        final_msg.header.stamp = node.get_clock().now().to_msg()
        final_msg.header.frame_id = "base_link"
        final_actuation = ActuationCommand(accel_cmd=0.0, brake_cmd=0.5, steer_cmd=0.0)
        final_msg.actuation = final_actuation
        node.actuation_pub.publish(final_msg)
        node.get_logger().info("Node shutting down. Sent final brake command.")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()