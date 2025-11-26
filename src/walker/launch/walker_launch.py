"""
@file walker_launch.py
@brief Launches Webots e-puck simulation, walker controller, and optional rosbag recording.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch argument to enable/disable rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="Enable rosbag recording (true/false)"
    )

    record_flag = LaunchConfiguration('record_bag')

    # Launch Webots simulation (e-puck)
    webots_pkg_dir = get_package_share_directory("webots_ros2_epuck")
    webots_launch_file = os.path.join(webots_pkg_dir, "launch", "robot_launch.py")

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(webots_launch_file)
    )

    # Launch walker controller node
    walker_node = Node(
        package="walker",
        executable="controller",
        name="walker_controller",
        output="screen"
    )
    
    # Declare an argument for specifying the output directory where bag files will be stored.
    # Only required if recording is enabled (condition = IfCondition(record_flag)).
    output_dir = DeclareLaunchArgument(
        'output_dir',
        condition = IfCondition(record_flag),
        description='Directory to store the recorded bag files'
    )
    
    # Execute rosbag2 recording as an external process.
    # Runs only if "record" == true.
    # Records all topics (-a) into the directory specified by output_dir.
    record_bag = ExecuteProcess(
        condition=IfCondition(record_flag),
        cmd=['ros2', 'bag', 'record', '-x', '/camera/.*','-a', '-o', LaunchConfiguration('output_dir')],
        output='screen'
    )

    return LaunchDescription([
        record_bag_arg,
        webots_launch,
        walker_node,
        output_dir,
        record_bag
    ])
