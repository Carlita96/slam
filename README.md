# Slam

Project to implement a SLAM for Lidar L515 of Intel.

## Installation

This project runs on Ubuntu 22.04. The steps for installing it are:
1. Install ROS2 by following this tutorial: [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
1. Install colcon by following this tutorial: [Colcon Installation](https://colcon.readthedocs.io/en/released/user/installation.html)
1. Install the latest Intel Realsense SDK: `sudo apt install ros-humble-librealsense2*`
1. Install the latest Intel Realsense ROS2 Wrapper: `sudo apt install ros-humble-realsense2-*`
1. Create a ROS2 workspace: `mkdir -p ~/ros2_ws/src`
1. Clone this repository: `cd ~/ros2_ws/src && git clone https://github.com/Carlita96/slam.git`
1. Build the workspace: `cd ~/ros2_ws && colcon build --symlink-install`

## Run example

To run the example connecting to a Lidar L515 and visualizing the data, run the following command:
`ros2 launch slam visualize_lidar_data.py`

To run the example with recorded rosbag, run the following command:
`ros2 launch slam visualize_rosbag_data.py rosbag_filepath:=<full path to rosbags DB3 file>`
For example:
`ros2 launch slam visualize_rosbag_data.py rosbag_filepath:=/home/user/path/to/file.db3`