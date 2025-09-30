#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.publisher = self.create_publisher(JointTrajectory, 
                                               '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info("GripperController started")
        self.send_grasp_command()

    def send_grasp_command(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'joint_lift',
            'joint_wrist_yaw',
            'joint_wrist_pitch',
            'joint_wrist_roll',
            'joint_gripper_slide',
            'joint_head_pan',
            'joint_head_tilt'
        ]

        point = JointTrajectoryPoint()
        # Example positions: adjust based on your grasp pose
        point.positions = [
            0.4,   # joint_lift
            0.0,   # joint_wrist_yaw
            -0.5,  # joint_wrist_pitch
            0.0,   # joint_wrist_roll
            0.02,  # joint_gripper_slide (close gripper)
            0.0,   # joint_head_pan
            -0.1   # joint_head_tilt
        ]
        point.time_from_start.sec = 2  # move in 2 seconds
        point.time_from_start.nanosec = 0

        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info("Published grasp trajectory")

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
