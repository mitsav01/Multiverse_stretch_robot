#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')

        # Publisher for base velocity
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber for odometry
        self.pose_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Target in odom frame (set your goal here)
        self.target_x = 1.9
        self.target_y = -0.3
        self.goal_tolerance = 0.05

        self.current_pose = None
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.current_pose is None:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info("✅ Goal reached!")
            return

        # Compute angle to goal
        angle_to_goal = math.atan2(dy, dx)
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_error = angle_to_goal - yaw

        # Normalize angle error
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Simple proportional control
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = 0.2 * distance
        cmd.twist.angular.z = 0.8 * angle_error
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.cmd_pub.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
