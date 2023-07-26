#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraSlamNode : public rclcpp::Node {
public:
    CameraSlamNode() : Node("camera_slam_node") {
        // Create the subscriber to /camera/color/image_raw topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&CameraSlamNode::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Log the image received (you can process the image here if needed)
        RCLCPP_INFO(this->get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSlamNode>());
    rclcpp::shutdown();
    return 0;
}
