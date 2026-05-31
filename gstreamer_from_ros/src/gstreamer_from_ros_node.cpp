#include "gstreamer_from_ros/gstreamer_from_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace gstreamer_from_ros {

GStreamerFromRos::GStreamerFromRos(const rclcpp::NodeOptions& options)
    : Node("gstreamer_from_ros_node", options),
      pipeline_(nullptr),
      appsrc_(nullptr),
      enc_capsfilter_(nullptr),
      bus_(nullptr),
      pipeline_started_(false),
      pipeline_error_(false) {
    gst_init(nullptr, nullptr);

    input_topic_ = declare_parameter<std::string>("input_topic", "");
    destination_ip_ = declare_parameter<std::string>("destination_ip", "");
    destination_port_ = declare_parameter<int>("destination_port", 5000);
    bitrate_ = declare_parameter<int>("bitrate", 500000);
    preset_level_ = declare_parameter<int>("preset_level", 1);
    iframe_interval_ = declare_parameter<int>("iframe_interval", 15);
    control_rate_ = declare_parameter<int>("control_rate", 1);
    pt_ = declare_parameter<int>("pt", 96);
    config_interval_ = declare_parameter<int>("config_interval", 1);
    expected_input_fps_ = declare_parameter<int>("expected_input_fps", 15);
    input_format_ = declare_parameter<std::string>("input_format", "RGB");
    hw_encoder_ = declare_parameter<bool>("hw_encoder", true);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, qos,
        std::bind(&GStreamerFromRos::imageCb, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
        RCLCPP_INFO(get_logger(), "Waiting for images on topic: '%s'",
                    input_topic_.c_str());
    });

    bus_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                   std::bind(&GStreamerFromRos::drain_bus, this));

    create_pipeline();
}

GStreamerFromRos::~GStreamerFromRos() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
    if (bus_) {
        gst_object_unref(bus_);
    }
}

void GStreamerFromRos::drain_bus() {
    if (!bus_) return;

    GstMessage* msg;
    while ((msg = gst_bus_pop(bus_)) != nullptr) {
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_error(msg, &err, &debug);
                RCLCPP_ERROR(get_logger(), "GStreamer error: %s (%s)",
                             err->message, debug ? debug : "no debug info");
                g_clear_error(&err);
                g_free(debug);
                pipeline_error_ = true;
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError* err = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_warning(msg, &err, &debug);
                // "code not implemented" is a known benign GStreamer quirk when
                // x265enc uses main-444 profile (e.g. GRAY8 input via videoconvert).
                if (g_strstr_len(err->message, -1, "code not implemented")) {
                    RCLCPP_DEBUG(get_logger(), "GStreamer: %s", err->message);
                } else {
                    RCLCPP_WARN(get_logger(), "GStreamer warning: %s", err->message);
                }
                g_clear_error(&err);
                g_free(debug);
                break;
            }
            default:
                break;
        }
        gst_message_unref(msg);
    }
}

void GStreamerFromRos::create_pipeline() {
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

    // capsfilter forces I420 so x265enc always uses main profile rather than
    // main-444. Odd-width padding is handled in imageCb (C++ side) so we avoid
    // relying on videoscale, which silently zeroes luma for this dimension case.
    if (!hw_encoder_) {
        enc_capsfilter_ = gst_element_factory_make("capsfilter", "enc_caps");
    }

    bool elements_ok = appsrc_ && convert && encoder && parser && pay && sink && pipeline_;
    if (hw_encoder_) elements_ok = elements_ok && nvconv;
    if (!hw_encoder_) elements_ok = elements_ok && enc_capsfilter_;

    if (!elements_ok) {
        RCLCPP_FATAL(get_logger(), "Failed to create GStreamer elements");
        return;
    }

    // Cap the appsrc queue so a slow encoder cannot cause unbounded memory growth.
    // max-bytes=0 disables the byte limit; max-buffers=4 drops frames beyond 4
    // queued when the encoder falls behind (block=FALSE = drop, not block).
    g_object_set(appsrc_,
        "max-bytes", guint64(0),
        "max-buffers", guint64(4),
        "block", FALSE,
        NULL);

    if (hw_encoder_) {
        g_object_set(encoder, "bitrate", bitrate_, "preset-level", preset_level_,
                     "iframeinterval", iframe_interval_, "control-rate", control_rate_, NULL);
    } else {
        // x265enc bitrate is in kbits/sec; key-int-max is the I-frame interval.
        g_object_set(encoder, "bitrate", bitrate_ / 1000,
                     "key-int-max", iframe_interval_,
                     "speed-preset", 1,  // ultrafast
                     NULL);
    }

    g_object_set(pay, "config-interval", config_interval_, "pt", pt_, NULL);
    g_object_set(sink, "host", destination_ip_.c_str(), "port", destination_port_, "sync", FALSE, NULL);

    if (hw_encoder_) {
        gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, nvconv, encoder,
                         parser, pay, sink, NULL);
        if (!gst_element_link_many(appsrc_, convert, nvconv, encoder, parser, pay, sink, NULL)) {
            RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
            return;
        }
    } else {
        gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, enc_capsfilter_,
                         encoder, parser, pay, sink, NULL);
        if (!gst_element_link_many(appsrc_, convert, enc_capsfilter_,
                                   encoder, parser, pay, sink, NULL)) {
            RCLCPP_FATAL(get_logger(), "Pipeline linking failed");
            return;
        }
    }

    bus_ = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));

    RCLCPP_INFO(get_logger(), "GStreamer H.265 pipeline created (%s encoder)",
                hw_encoder_ ? "NVIDIA hw" : "x265 sw");
}

void GStreamerFromRos::imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    static size_t frame_count = 0;
    frame_count++;

    if (pipeline_error_) return;

    // x265 requires even width/height. Pad odd dimensions by duplicating the
    // last column/row in C++ rather than using videoscale, which zeroes luma.
    const int aw = (static_cast<int>(msg->width)  + 1) & ~1;
    const int ah = (static_cast<int>(msg->height) + 1) & ~1;
    const size_t bpp = msg->step / msg->width;  // bytes per pixel

    if (!pipeline_started_) {
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw", "format", G_TYPE_STRING, input_format_.c_str(),
            "width", G_TYPE_INT, aw,
            "height", G_TYPE_INT, ah,
            "framerate", GST_TYPE_FRACTION, expected_input_fps_, 1, NULL);

        g_object_set(appsrc_, "caps", caps, "format", GST_FORMAT_TIME,
                     "is-live", TRUE, "do-timestamp", TRUE, NULL);
        gst_caps_unref(caps);

        if (enc_capsfilter_) {
            GstCaps* enc_caps = gst_caps_new_simple(
                "video/x-raw", "format", G_TYPE_STRING, "I420",
                "width", G_TYPE_INT, aw, "height", G_TYPE_INT, ah, NULL);
            g_object_set(enc_capsfilter_, "caps", enc_caps, NULL);
            gst_caps_unref(enc_caps);
        }

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        pipeline_started_ = true;
        timer_->cancel();

        RCLCPP_INFO(get_logger(), "H.265 pipeline started (%s) %dx%d -> %dx%d %s",
                    hw_encoder_ ? "NVIDIA hw encoder" : "x265 sw encoder",
                    msg->width, msg->height, aw, ah, input_format_.c_str());
    }

    GstBuffer* buffer;
    if (aw == static_cast<int>(msg->width) && ah == static_cast<int>(msg->height)) {
        buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
    } else {
        // Pad to even dimensions: copy each row then duplicate last pixel/row.
        const size_t row_bytes = aw * bpp;
        std::vector<uint8_t> padded(row_bytes * ah);
        for (uint32_t r = 0; r < msg->height; r++) {
            const uint8_t* src = msg->data.data() + r * msg->step;
            uint8_t* dst = padded.data() + r * row_bytes;
            std::copy(src, src + msg->width * bpp, dst);
            if (aw > static_cast<int>(msg->width))
                std::copy(src + (msg->width - 1) * bpp, src + msg->width * bpp,
                          dst + msg->width * bpp);
        }
        if (ah > static_cast<int>(msg->height)) {
            const uint8_t* last = padded.data() + (msg->height - 1) * row_bytes;
            std::copy(last, last + row_bytes, padded.data() + msg->height * row_bytes);
        }
        buffer = gst_buffer_new_allocate(nullptr, padded.size(), nullptr);
        gst_buffer_fill(buffer, 0, padded.data(), padded.size());
    }

    // gst_app_src_push_buffer takes ownership of buffer; do NOT unref after.
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);

    if (ret != GST_FLOW_OK)
        RCLCPP_WARN(get_logger(), "Frame #%zu dropped (encoder queue full or pipeline not ready)",
                    frame_count);
    else
        RCLCPP_DEBUG(get_logger(), "Pushed frame #%zu into GStreamer", frame_count);
}

}  // namespace gstreamer_from_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_from_ros::GStreamerFromRos)
