#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Move(Node):
    def __init__(self):
        super().__init__('stretch_base_move')  # Name of the node
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        self.get_logger().info("Starting to move in a circle...")

        timer_period = 0.5  # seconds (2 Hz)
        self.timer = self.create_timer(timer_period, self.move_around)

    def move_around(self):
        command = TwistStamped()

        # Fill header
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'

        # No linear motion
        command.twist.linear.x = 0.0
        command.twist.linear.y = 0.0
        command.twist.linear.z = 0.0

        # Only rotation around Z (yaw)
        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0
        command.twist.angular.z = 0.5  # Rotate in place

        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    base_motion = Move()
    rclpy.spin(base_motion)
    base_motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
