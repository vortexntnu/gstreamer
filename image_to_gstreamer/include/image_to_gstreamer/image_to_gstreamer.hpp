#ifndef IMAGE_TO_GSTREAMER__IMAGE_TO_GSTREAMER_HPP_
#define IMAGE_TO_GSTREAMER__IMAGE_TO_GSTREAMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class ImageToGStreamer : public rclcpp::Node
{
public:
  explicit ImageToGStreamer();
  ~ImageToGStreamer();

private:
  void create_pipeline();
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int bitrate_;
  int preset_level_;
  int iframe_interval_;
  int control_rate_;
  int pt_;
  int config_interval_;
  int framerate_;

  GstElement *pipeline_;
  GstElement *appsrc_;

  bool pipeline_started_;

  std::string input_topic_;
  std::string host_;
  int port_;
};

#endif  // IMAGE_TO_GSTREAMER__IMAGE_TO_GSTREAMER_HPP_