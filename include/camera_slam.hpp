#ifndef CAMERA_SLAM_HPP_
#define CAMERA_SLAM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraSlamNode : public rclcpp::Node {
public:
    CameraSlamNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

#endif // CAMERA_SLAM_HPP_
