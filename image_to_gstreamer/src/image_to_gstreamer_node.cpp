#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class ImageToGStreamer : public rclcpp::Node
{
public:
  ImageToGStreamer()
  : Node("image_to_gstreamer_node")
  {
    gst_init(nullptr, nullptr);

    input_topic_ = this->declare_parameter<std::string>("input_topic", "/cam/image_color");

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImageToGStreamer::imageCb, this, std::placeholders::_1));

    create_pipeline();
  }

  ~ImageToGStreamer()
  {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  GstElement *pipeline_;
  GstElement *appsrc_;

  bool pipeline_started_ = false;

  void create_pipeline()
  {
    pipeline_ = gst_pipeline_new("ros2-h265-pipeline");

    appsrc_ = gst_element_factory_make("appsrc", "source");
    GstElement *convert = gst_element_factory_make("videoconvert", "convert");
    GstElement *encoder = gst_element_factory_make("nvh265enc", "encoder");
    GstElement *parser  = gst_element_factory_make("h265parse", "parser");
    GstElement *pay     = gst_element_factory_make("rtph265pay", "pay");
    GstElement *sink    = gst_element_factory_make("udpsink", "sink");

    if (!appsrc_ || !convert ||
        !encoder || !parser || !pay || !sink)
    {
      if (!appsrc_)
         RCLCPP_FATAL(get_logger(), "Failed to create appsrc");
      if (!convert)
         RCLCPP_FATAL(get_logger(), "Failed to create convert");
      if (!encoder)
         RCLCPP_FATAL(get_logger(), "Failed to create encoder");
      if (!parser)
         RCLCPP_FATAL(get_logger(), "Failed to create parser");
      if (!pay)
         RCLCPP_FATAL(get_logger(), "Failed to create pay");
      if (!sink)
         RCLCPP_FATAL(get_logger(), "Failed to create sink");

      RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
      return;
    }
    if (!pipeline_) {
      RCLCPP_FATAL(get_logger(), "Pipeline creation failed");
      return;
    }

    g_object_set(sink,
      "host", "127.0.0.1",
      "port", 5000,
      "sync", FALSE,
      NULL);

    g_object_set(encoder,
    "bitrate", 50000,   // 4 Mbps
    "preset", 3,          // low-latency
    "rc-mode", 1,         // CBR
    "gop-size", 30,
    NULL);

    g_object_set(pay,
        "config-interval", 1,  // send SPS/PPS every 1 second
        "pt", 96,              // payload type
    NULL);

    gst_bin_add_many(GST_BIN(pipeline_),
      appsrc_, convert, encoder, parser, pay, sink, NULL);

    if (!gst_element_link_many(
        appsrc_, convert, encoder, parser, pay, sink, NULL))
    {
      RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
      return;
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!pipeline_started_)
    {
      GstCaps *caps = gst_caps_new_simple(
          "video/x-raw",
          "format", G_TYPE_STRING, "RGB",
          "width", G_TYPE_INT, msg->width,
          "height", G_TYPE_INT, msg->height,
          "framerate", GST_TYPE_FRACTION, 30, 1,
          NULL);
      g_object_set(appsrc_,
          "caps", caps,
          "format", GST_FORMAT_TIME,
          "is-live", TRUE,
          "do-timestamp", TRUE,
          NULL);
      gst_caps_unref(caps);

      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      pipeline_started_ = true;

      RCLCPP_INFO(get_logger(), "H.265 GPU pipeline started");
    }

    GstBuffer *buffer = gst_buffer_new_allocate(
      nullptr, msg->data.size(), nullptr);

    gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
      RCLCPP_WARN(get_logger(), "Failed to push buffer");
  }

  // ----------------------------- Params & ROS --------------------------------
  // Topics
  std::string input_topic_{"/cam/image_color"};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageToGStreamer>());
  rclcpp::shutdown();
  return 0;
}