import os
from ament_index_python.packages import get_packages_with_prefixes

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 

def generate_launch_description():
    package_name = 'slam'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    launch_description = LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(pkg_share, 'config', 'config_file.rviz')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py'
            ]),
            launch_arguments={
                'depth_module.profile': 'high_accuracy',
                'pointcloud.enable': 'true'
            }.items()
        )
    ])

    return launch_description