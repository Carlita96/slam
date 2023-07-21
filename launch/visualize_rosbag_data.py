import os

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():
    # Get the path to slam package share directory
    package_name = 'slam'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Read the rosbag filepath from the command line
    rosbag_filepath = LaunchConfiguration('rosbag_filepath')
    rosbag_filepath_arg = DeclareLaunchArgument(
        "rosbag_filepath", 
        description='Full path to the rosbag file to be played back.'
    )

    # Define the launch description
    launch_description = LaunchDescription([
        rosbag_filepath_arg,
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'config_file.rviz')]
        ),
        ExecuteProcess(cmd=[
                'ros2',
                'bag',
                'play',
                '--loop',
                rosbag_filepath
            ],
            output='screen')
    ])

    return launch_description