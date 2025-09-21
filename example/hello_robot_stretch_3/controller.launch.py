import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Directory of this launch file
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    # Paths to URDF and YAML
    urdf_path = os.path.join(launch_dir, 'stretch.urdf')
    controller_yaml = os.path.join(launch_dir, 'config', 'stretch_ros2_control.yaml')
    rviz_config = os.path.join(launch_dir, 'config', 'stretch.rviz')

    # Read URDF
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    # Nodes
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
        remappings=[
        ('/diff_drive_controller/cmd_vel', '/cmd_vel')
    ]
    )

    # Spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    omni_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_drive_controller'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Ensure controllers spawn in sequence
    # joint_trajectory_controller spawns after joint_state_broadcaster
    # joint_trajectory_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[joint_trajectory_controller_spawner]
    #     )
    # )

    # # diff_drive_controller spawns after joint_trajectory_controller
    # diff_drive_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_trajectory_controller_spawner,
    #         on_exit=[diff_drive_controller_spawner]
    #     )
    # )


    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        #omni_drive_controller_spawner,

        #diff_drive_controller_spawner,
        #static_tf_node
        # joint_trajectory_handler,
        # diff_drive_handler,
    ])
