#include "gstreamer_to_ros/gstreamer_to_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace gstreamer_to_ros {

GStreamerToROS::GStreamerToROS(const rclcpp::NodeOptions& options)
    : Node("gstreamer_to_ros_node", options), pipeline_(nullptr), appsink_(nullptr) {
    gst_init(nullptr, nullptr);

    host_ = declare_parameter<std::string>("host", "0.0.0.0");
    port_ = declare_parameter<int>("port", 5001);
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/image_raw");
    output_format_ = declare_parameter<std::string>("output_format", "BGR");
    hw_decoder_ = declare_parameter<bool>("hw_decoder", true);

    pub_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, rclcpp::SensorDataQoS().keep_last(1));

    create_pipeline();
}

GStreamerToROS::~GStreamerToROS() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

void GStreamerToROS::create_pipeline() {
    pipeline_ = gst_pipeline_new("h265-receive-pipeline");

    GstElement* src = gst_element_factory_make("udpsrc", "src");
    GstElement* depay = gst_element_factory_make("rtph265depay", "depay");
    GstElement* parse = gst_element_factory_make("h265parse", "parse");
    GstElement* convert = gst_element_factory_make("videoconvert", "convert");
    appsink_ = gst_element_factory_make("appsink", "sink");

    GstElement* decoder = hw_decoder_
        ? gst_element_factory_make("nvh265dec", "decoder")
        : gst_element_factory_make("avdec_h265", "decoder");

    if (!pipeline_ || !src || !depay || !parse || !decoder || !convert || !appsink_) {
        RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
        return;
    }

    GstCaps* caps = gst_caps_new_simple(
        "application/x-rtp", "media", G_TYPE_STRING, "video",
        "encoding-name", G_TYPE_STRING, "H265",
        "payload", G_TYPE_INT, 96, NULL);

    g_object_set(src, "port", port_, "caps", caps, NULL);
    gst_caps_unref(caps);

    // Force videoconvert to produce the requested output format so the ROS
    // message encoding and step are always consistent with the buffer contents.
    GstCaps* out_caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, output_format_.c_str(), NULL);
    g_object_set(appsink_, "emit-signals", TRUE, "sync", FALSE, "caps", out_caps, NULL);
    gst_caps_unref(out_caps);
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(GStreamerToROS::on_new_sample), this);

    gst_bin_add_many(GST_BIN(pipeline_), src, depay, parse, decoder, convert, appsink_, NULL);

    if (!gst_element_link_many(src, depay, parse, decoder, convert, appsink_, NULL)) {
        RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
        return;
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    RCLCPP_INFO(get_logger(), "H.265 UDP receiver pipeline started on port %d (%s decoder)",
                port_, hw_decoder_ ? "NVIDIA hw" : "avdec sw");
}

GstFlowReturn GStreamerToROS::on_new_sample(GstAppSink* sink, gpointer user_data) {
    auto* node = static_cast<GStreamerToROS*>(user_data);

    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample)
        return GST_FLOW_ERROR;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* structure = gst_caps_get_structure(caps, 0);

    int width = 0, height = 0;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const std::string fmt = gst_structure_get_string(structure, "format");

    sensor_msgs::msg::Image msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "camera";
    msg.width = width;
    msg.height = height;
    if (fmt == "GRAY8") {
        msg.encoding = "mono8";
        msg.step = width;
    } else {
        msg.encoding = "bgr8";
        msg.step = width * 3;
    }
    msg.data.assign(map.data, map.data + map.size);

    node->pub_->publish(msg);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

}  // namespace gstreamer_to_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_to_ros::GStreamerToROS)
