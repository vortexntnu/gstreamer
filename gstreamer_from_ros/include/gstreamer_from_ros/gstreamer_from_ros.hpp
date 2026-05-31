#ifndef GSTREAMER_FROM_ROS__GSTREAMER_FROM_ROS_HPP_
#define GSTREAMER_FROM_ROS__GSTREAMER_FROM_ROS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

namespace gstreamer_from_ros {

class GStreamerFromRos : public rclcpp::Node {
   public:
    explicit GStreamerFromRos(const rclcpp::NodeOptions& options);
    ~GStreamerFromRos();

   private:
    void create_pipeline();
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);
    void drain_bus();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr bus_timer_;
    int bitrate_;
    int preset_level_;
    int iframe_interval_;
    int control_rate_;
    int pt_;
    int config_interval_;
    int expected_input_fps_;
    std::string input_format_;
    bool hw_encoder_;

    GstElement* pipeline_;
    GstElement* appsrc_;
    GstElement* enc_capsfilter_;
    GstBus*     bus_;

    bool pipeline_started_;
    bool pipeline_error_;

    std::string input_topic_;
    std::string destination_ip_;
    int destination_port_;
};

}  // namespace gstreamer_from_ros

#endif  // GSTREAMER_FROM_ROS__GSTREAMER_FROM_ROS_HPP_
