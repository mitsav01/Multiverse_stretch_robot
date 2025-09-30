#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pynput import keyboard


class StretchTeleop(Node):
    def __init__(self):
        super().__init__('stretch_teleop_full')
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',  # adjust if your controller uses a different topic
            10
        )

        self.get_logger().info(
            "Keyboard teleop started.\n"
            "W/S: forward/backward\n"
            "A/D: rotate left/right\n"
            "Q/E: alternative rotation left/right\n"
            "X: stop\n"
            "+/-: adjust linear/angular speed\n"
            "Press Ctrl+C to exit"
        )

        # Speeds
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        # Key buffer for simultaneous key presses
        self.key_buffer = set()

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        if isinstance(key, keyboard.KeyCode):
            self.key_buffer.add(key.char.lower())

    def on_release(self, key):
        if isinstance(key, keyboard.KeyCode):
            self.key_buffer.discard(key.char.lower())

    def publish_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # Reset motion each cycle
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        # Movement
        if 'w' in self.key_buffer:
            cmd.twist.linear.x += self.linear_speed
        if 's' in self.key_buffer:
            cmd.twist.linear.x -= self.linear_speed

        # Rotation
        if 'a' in self.key_buffer or 'q' in self.key_buffer:
            cmd.twist.angular.z += self.angular_speed
        if 'd' in self.key_buffer or 'e' in self.key_buffer:
            cmd.twist.angular.z -= self.angular_speed

        # Stop
        if 'x' in self.key_buffer:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        # Speed scaling
        if '+' in self.key_buffer:
            self.linear_speed = min(self.linear_speed + 0.05, 1.0)
            self.angular_speed = min(self.angular_speed + 0.1, 2.0)
        if '-' in self.key_buffer:
            self.linear_speed = max(self.linear_speed - 0.05, 0.05)
            self.angular_speed = max(self.angular_speed - 0.1, 0.1)

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StretchTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Teleop stopped.")
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
