import os

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to slam package share directory
    package_name = 'slam'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # Set the GAZEBO_MODEL_PATH to include the directory of the concrete_floor model
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":{}/models".format(pkg_share)
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "{}/models".format(pkg_share)

    # Get the path to the URDF and Gazebo world files
    robot_description_path = os.path.join(
        pkg_share,
        'urdf',
        'model.xml'
    )
    world_file_path = os.path.join(
        pkg_share,
        'worlds',
        'house.world'
    )

    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Arguments for the robot state publisher
    urdf_file = LaunchConfiguration('urdf_file', default=robot_description_path)

    # Arguments for the Gazebo simulation
    headless_mode = LaunchConfiguration('headless', default='false')
    gazebo_server = LaunchConfiguration('gazebo_server', default='false')
    world_file = LaunchConfiguration('world_file', default=world_file_path)

    # Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare(package='gazebo_ros').find('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        )),
        launch_arguments={
            'headless': headless_mode,
            'server_required': gazebo_server,
            'world': world_file
        }.items()
    )

    # Spawning the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'r2d2', '-file', urdf_file, '-robot_namespace', 
                   'robot_namespace', '-x', '0', '-y', '0', '-z', '1'],
        output='screen'
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=[urdf_file]
    )

    # SLAM Nodes
    camera_slam_node = Node(
        package='slam',
        namespace='slam',
        executable='camera_slam_node',
        name='camera_slam_node'
    )
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'config_file.rviz')]
    )


    # Define the launch description
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=robot_description_path,
            description='Path to the robot URDF file'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Whether to run Gazebo in headless mode (true/false)'
        ),
        DeclareLaunchArgument(
            'gazebo_server',
            default_value='false',
            description='Whether to run the Gazebo server separately (true/false)'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=world_file_path,
            description='Path to the Gazebo world file'
        ),
        camera_slam_node,
        # rviz_node,
        gazebo_launch,
        spawn_entity,
        robot_state_publisher_node
    ])

    return launch_description