import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    # URDF and config paths
    urdf_path = os.path.join(launch_dir, 'stretch.urdf')
    controller_yaml = os.path.join(launch_dir, 'config', 'stretch_ros2_control.yaml')
    rviz_config = os.path.join(launch_dir, 'config', 'stretch.rviz')
    scene_xml_path = os.path.join(launch_dir, "scene_two_cups.xml")

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    # ROS2 nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml],
        output='screen',
        remappings=[('/diff_drive_controller/cmd_vel', '/cmd_vel')]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    joint_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_velocity_controller'],
        output='both'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='both'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        diff_drive_controller_spawner,
    ])
