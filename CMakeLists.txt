cmake_minimum_required(VERSION 3.5)
project(slam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necesary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# Xacro
file(GLOB xacro_files urdf/*.urdf.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND urdf_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${urdf_files})

# Add the executable targets
add_executable(camera_slam_node src/camera_slam.cpp)
ament_target_dependencies(camera_slam_node rclcpp std_msgs sensor_msgs)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Install these targets in the lib folder
install(TARGETS
  talker
  listener
  camera_slam_node
  DESTINATION lib/${PROJECT_NAME})

# Install the config and launch files in the share folder
install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME})

# Make sure the headers are available to other ROS2 packages
install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()