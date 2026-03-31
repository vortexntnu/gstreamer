#ifndef GSTREAMER_TO_ROS__GSTREAMER_TO_ROS_HPP_
#define GSTREAMER_TO_ROS__GSTREAMER_TO_ROS_HPP_

#include <string>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class GStreamerToROS : public rclcpp::Node {
   public:
    GStreamerToROS();
    ~GStreamerToROS();

   private:
    void create_pipeline();

    static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    GstElement* pipeline_;
    GstElement* appsink_;

    std::string host_;
    int port_;
    std::string output_topic_;
};

#endif  // GSTREAMER_TO_ROS__GSTREAMER_TO_ROS_HPP_
