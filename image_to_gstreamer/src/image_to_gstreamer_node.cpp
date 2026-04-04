#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include "image_to_gstreamer/image_to_gstreamer.hpp"
#include <vortex/utils/ros/qos_profiles.hpp>

ImageToGStreamer::ImageToGStreamer()
    : Node("image_to_gstreamer_node"),
      pipeline_(nullptr),
      appsrc_(nullptr),
      pipeline_started_(false) {
    gst_init(nullptr, nullptr);

    // Correct member initialization
    input_topic_ = this->declare_parameter<std::string>("input_topic", "");
    host_ = this->declare_parameter<std::string>("host", "");
    port_ = this->declare_parameter<int>("port", 5000);  // must match receiver

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&ImageToGStreamer::imageCb, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
        RCLCPP_INFO(this->get_logger(), "Waiting for images on topic: '%s'",
                    input_topic_.c_str());
    });

    bitrate_ = this->declare_parameter<int>("bitrate", 500000);   // bitrate bit
    preset_level_ = this->declare_parameter<int>("preset_level", 1);    // 1-4 1 = low latancy low quality 4 = high latency high qulity
    iframe_interval_ = this->declare_parameter<int>("iframe_interval", 15);  // how many control framse is sent (full frames)
    control_rate_ = this->declare_parameter<int>("control_rate", 1);  // 0 = fixed bitrate , 1 = variabel with target, 2 = constant QP, 3 = variabel with celing
    pt_ = this->declare_parameter<int>("pt", 96);  // RTP Payload Type number defaut value
    config_interval_ = this->declare_parameter<int>("config_interval", 1); // keep at 1
    framerate_ = this->declare_parameter<int>("framerate", 15); // you know what framrate is
    format = this->declare_parameter<std::string>("format", "RGB8");

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
    GstElement* nvconv = gst_element_factory_make("nvvidconv", "nvconv");
    GstElement* encoder = gst_element_factory_make("nvv4l2h265enc", "encoder");
    GstElement* parser = gst_element_factory_make("h265parse", "parser");
    GstElement* pay = gst_element_factory_make("rtph265pay", "pay");
    GstElement* sink = gst_element_factory_make("udpsink", "sink");

    if (!appsrc_ || !convert || !nvconv || !encoder || !parser || !pay ||
        !sink || !pipeline_) {
        RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
        return;
    }

    // Encoder properties for low-latency
    //configurs h265 encoder
    g_object_set(encoder, "bitrate", bitrate_, "preset-level", preset_level_,
                 "iframeinterval", iframe_interval_, "control-rate",
                 control_rate_, NULL);
    
    //configures packaging
    g_object_set(pay, "config-interval", config_interval_, "pt", pt_, NULL);
    
    //configures where to send it
    g_object_set(sink, "host", host_.c_str(), "port", port_, "sync", FALSE,
                 NULL);

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, nvconv, encoder,
                     parser, pay, sink, NULL);

    if (!gst_element_link_many(appsrc_, convert, nvconv, encoder, parser, pay,
                               sink, NULL)) {
        RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
        return;
    }
}

void ImageToGStreamer::imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    static size_t frame_count = 0;
    frame_count++;

    if (!pipeline_started_) {
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw", "format", G_TYPE_STRING, format.c_str(), "width", G_TYPE_INT,
            msg->width, "height", G_TYPE_INT, msg->height, "framerate",
            GST_TYPE_FRACTION, framerate_, 1, NULL);

        g_object_set(appsrc_, "caps", caps, "format", GST_FORMAT_TIME,
                     "is-live", TRUE, "do-timestamp", TRUE, NULL);
        gst_caps_unref(caps);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        pipeline_started_ = true;
        timer_->cancel();

        RCLCPP_INFO(get_logger(), "H.265 GPU pipeline started");
    }

    GstBuffer* buffer =
        gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
        RCLCPP_WARN(get_logger(), "Failed to push buffer");
    else
        RCLCPP_DEBUG(get_logger(), "Pushed frame #%zu into GStreamer",
                     frame_count);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToGStreamer>());
    rclcpp::shutdown();
    return 0;
}
