#include <rclcpp/rclcpp.hpp>
#include "image_to_gstreamer/image_to_gstreamer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToGStreamer>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
