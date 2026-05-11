#include "image_to_gstreamer/image_to_gstreamer.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace image_to_gstreamer {

ImageToGStreamer::ImageToGStreamer(const rclcpp::NodeOptions& options)
    : Node("image_to_gstreamer_node", options),
      pipeline_(nullptr),
      appsrc_(nullptr),
      pipeline_started_(false) {
    gst_init(nullptr, nullptr);

    input_topic_ = declare_parameter<std::string>("input_topic", "");
    host_ = declare_parameter<std::string>("host", "");
    port_ = declare_parameter<int>("port", 5000);
    bitrate_ = declare_parameter<int>("bitrate", 500000);
    preset_level_ = declare_parameter<int>("preset_level", 1);
    iframe_interval_ = declare_parameter<int>("iframe_interval", 15);
    control_rate_ = declare_parameter<int>("control_rate", 1);
    pt_ = declare_parameter<int>("pt", 96);
    config_interval_ = declare_parameter<int>("config_interval", 1);
    framerate_ = declare_parameter<int>("framerate", 15);
    format_ = declare_parameter<std::string>("format", "RGB");
    hw_encoder_ = declare_parameter<bool>("hw_encoder", true);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&ImageToGStreamer::imageCb, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
        RCLCPP_INFO(get_logger(), "Waiting for images on topic: '%s'",
                    input_topic_.c_str());
    });

    create_pipeline();
}

ImageToGStreamer::~ImageToGStreamer() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

void ImageToGStreamer::create_pipeline() {
    pipeline_ = gst_pipeline_new("ros2-h265-pipeline");
    appsrc_ = gst_element_factory_make("appsrc", "source");
    GstElement* convert = gst_element_factory_make("videoconvert", "convert");
    GstElement* parser = gst_element_factory_make("h265parse", "parser");
    GstElement* pay = gst_element_factory_make("rtph265pay", "pay");
    GstElement* sink = gst_element_factory_make("udpsink", "sink");

    GstElement* encoder = nullptr;
    GstElement* nvconv = nullptr;

    if (hw_encoder_) {
        nvconv = gst_element_factory_make("nvvidconv", "nvconv");
        encoder = gst_element_factory_make("nvv4l2h265enc", "encoder");
    } else {
        encoder = gst_element_factory_make("x265enc", "encoder");
    }

    bool elements_ok = appsrc_ && convert && encoder && parser && pay && sink && pipeline_;
    if (hw_encoder_) elements_ok = elements_ok && nvconv;

    if (!elements_ok) {
        RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
        return;
    }

    if (hw_encoder_) {
        g_object_set(encoder, "bitrate", bitrate_, "preset-level", preset_level_,
                     "iframeinterval", iframe_interval_, "control-rate", control_rate_, NULL);
    } else {
        // x265enc bitrate is in kbits/sec; key-int-max is the I-frame interval
        g_object_set(encoder, "bitrate", bitrate_ / 1000,
                     "key-int-max", iframe_interval_,
                     "speed-preset", 0,  // ultrafast — minimise latency
                     NULL);
    }

    g_object_set(pay, "config-interval", config_interval_, "pt", pt_, NULL);
    g_object_set(sink, "host", host_.c_str(), "port", port_, "sync", FALSE, NULL);

    if (hw_encoder_) {
        gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, nvconv, encoder,
                         parser, pay, sink, NULL);
        if (!gst_element_link_many(appsrc_, convert, nvconv, encoder, parser, pay, sink, NULL)) {
            RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
            return;
        }
    } else {
        gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, pay, sink, NULL);
        if (!gst_element_link_many(appsrc_, convert, encoder, parser, pay, sink, NULL)) {
            RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
            return;
        }
    }

    RCLCPP_INFO(get_logger(), "GStreamer H.265 pipeline created (%s encoder)",
                hw_encoder_ ? "NVIDIA hw" : "x265 sw");
}

void ImageToGStreamer::imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    static size_t frame_count = 0;
    frame_count++;

    if (!pipeline_started_) {
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw", "format", G_TYPE_STRING, format_.c_str(),
            "width", G_TYPE_INT, msg->width,
            "height", G_TYPE_INT, msg->height,
            "framerate", GST_TYPE_FRACTION, framerate_, 1, NULL);

        g_object_set(appsrc_, "caps", caps, "format", GST_FORMAT_TIME,
                     "is-live", TRUE, "do-timestamp", TRUE, NULL);
        gst_caps_unref(caps);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        pipeline_started_ = true;
        timer_->cancel();

        RCLCPP_INFO(get_logger(), "H.265 pipeline started (%s)",
                    hw_encoder_ ? "NVIDIA hw encoder" : "x265 sw encoder");
    }

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
        RCLCPP_WARN(get_logger(), "Failed to push buffer");
    else
        RCLCPP_DEBUG(get_logger(), "Pushed frame #%zu into GStreamer", frame_count);
}

}  // namespace image_to_gstreamer

RCLCPP_COMPONENTS_REGISTER_NODE(image_to_gstreamer::ImageToGStreamer)
