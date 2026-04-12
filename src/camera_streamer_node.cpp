#include "bluerov2_controller/camera_streamer_node.hpp"

#include <chrono>

namespace bluerov2_controller
{

CameraStreamerNode::CameraStreamerNode(const rclcpp::NodeOptions & options)
: Node("camera_streamer", options)
{
  // Declare parameters
  port_ = this->declare_parameter("port", 5600);
  decoder_ = this->declare_parameter("decoder", std::string("auto"));
  resize_enable_ = this->declare_parameter("resize_enable", false);
  resize_width_ = this->declare_parameter("resize_width", 960);
  resize_height_ = this->declare_parameter("resize_height", 540);
  frame_id_ = this->declare_parameter("frame_id", std::string("bluerov2_camera"));
  watchdog_period_sec_ = this->declare_parameter("watchdog_period_sec", 2.0);
  max_watchdog_failures_ = this->declare_parameter("max_watchdog_failures", 3);


  // Initialize GStreamer
  gst_init(nullptr, nullptr);

  // Create image_transport publisher (provides /compressed, /theora variants)
  image_pub_ = image_transport::create_publisher(this, "camera/image");

  // Start pipeline
  if (!init_pipeline()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize GStreamer pipeline");
    return;
  }

  // Watchdog timer
  auto period = std::chrono::duration<double>(watchdog_period_sec_);
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CameraStreamerNode::watchdog_callback, this));

  RCLCPP_INFO(this->get_logger(), "Camera streamer started on UDP port %d with decoder '%s'",
              port_, decoder_.c_str());
}

CameraStreamerNode::~CameraStreamerNode()
{
  shutdown_pipeline();
}

// ---------------------------------------------------------------------------
// Decoder selection
// ---------------------------------------------------------------------------
std::string CameraStreamerNode::select_decoder() const
{
  if (decoder_ != "auto") {
    return decoder_;
  }

  // Try hardware decoders first, fall back to software
  const char * candidates[] = {"vaapih264dec", "nvh264dec"};
  for (const char * name : candidates) {
    GstElementFactory * factory = gst_element_factory_find(name);
    if (factory) {
      gst_object_unref(factory);
      RCLCPP_INFO(this->get_logger(), "Auto-selected hardware decoder: %s", name);
      return name;
    }
  }

  RCLCPP_INFO(this->get_logger(), "No hardware decoder found, using software avdec_h264");
  return "avdec_h264";
}

// ---------------------------------------------------------------------------
// Build GStreamer pipeline string
// ---------------------------------------------------------------------------
std::string CameraStreamerNode::build_pipeline_string() const
{
  std::string dec = select_decoder();

  std::string pipeline =
    "udpsrc port=" + std::to_string(port_) +
    " ! application/x-rtp,payload=96"
    " ! rtph264depay"
    " ! h264parse"
    " ! " + dec +
    " ! videoconvert"
    " ! video/x-raw,format=BGR"
    " ! appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true";

  RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline.c_str());
  return pipeline;
}

// ---------------------------------------------------------------------------
// Initialize GStreamer pipeline
// ---------------------------------------------------------------------------
bool CameraStreamerNode::init_pipeline()
{
  GError * error = nullptr;
  std::string pipeline_str = build_pipeline_string();

  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
  if (error) {
    RCLCPP_ERROR(this->get_logger(), "GStreamer parse error: %s", error->message);
    g_error_free(error);
    return false;
  }

  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
  if (!appsink_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  // Set appsink callbacks
  GstAppSinkCallbacks callbacks = {};
  callbacks.new_sample = &CameraStreamerNode::on_new_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

  // Start pipeline
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING");
    gst_object_unref(appsink_);
    gst_object_unref(pipeline_);
    appsink_ = nullptr;
    pipeline_ = nullptr;
    return false;
  }

  frame_count_ = 0;
  last_watchdog_count_ = 0;
  watchdog_failures_ = 0;

  return true;
}

// ---------------------------------------------------------------------------
// Shutdown pipeline
// ---------------------------------------------------------------------------
void CameraStreamerNode::shutdown_pipeline()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (appsink_) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

// ---------------------------------------------------------------------------
// GStreamer appsink callback (runs on GStreamer streaming thread)
// ---------------------------------------------------------------------------
GstFlowReturn CameraStreamerNode::on_new_sample(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<CameraStreamerNode *>(user_data);

  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) {
    return GST_FLOW_ERROR;
  }

  self->publish_frame(sample);
  gst_sample_unref(sample);

  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Publish frame as sensor_msgs::msg::Image
// ---------------------------------------------------------------------------
void CameraStreamerNode::publish_frame(GstSample * sample)
{
  GstCaps * caps = gst_sample_get_caps(sample);
  if (!caps) {
    return;
  }

  GstStructure * s = gst_caps_get_structure(caps, 0);
  int width = 0, height = 0;
  gst_structure_get_int(s, "width", &width);
  gst_structure_get_int(s, "height", &height);

  if (width <= 0 || height <= 0) {
    return;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    return;
  }

  auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
  img_msg->header.stamp = this->now();
  img_msg->header.frame_id = frame_id_;
  img_msg->encoding = "bgr8";
  img_msg->is_bigendian = false;

  if (resize_enable_ && (resize_width_ != width || resize_height_ != height)) {
    // Resize using OpenCV
    cv::Mat src(height, width, CV_8UC3, map.data);
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(resize_width_, resize_height_), 0, 0, cv::INTER_AREA);

    img_msg->width = resize_width_;
    img_msg->height = resize_height_;
    img_msg->step = resize_width_ * 3;
    img_msg->data.assign(dst.data, dst.data + dst.total() * dst.elemSize());
  } else {
    // Zero-copy path: direct buffer to message
    img_msg->width = width;
    img_msg->height = height;
    img_msg->step = width * 3;
    img_msg->data.assign(map.data, map.data + map.size);
  }

  gst_buffer_unmap(buffer, &map);

  image_pub_.publish(std::move(img_msg));
  frame_count_.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Watchdog: restart pipeline if no frames received
// ---------------------------------------------------------------------------
void CameraStreamerNode::watchdog_callback()
{
  uint64_t current_count = frame_count_.load(std::memory_order_relaxed);

  if (current_count == last_watchdog_count_) {
    watchdog_failures_++;
    RCLCPP_WARN(this->get_logger(), "No new frames received (%d/%d)",
                watchdog_failures_, max_watchdog_failures_);

    if (watchdog_failures_ >= max_watchdog_failures_) {
      RCLCPP_WARN(this->get_logger(), "Restarting GStreamer pipeline...");
      restart_pipeline();
    }
  } else {
    watchdog_failures_ = 0;
  }

  last_watchdog_count_ = current_count;
}

void CameraStreamerNode::restart_pipeline()
{
  shutdown_pipeline();

  if (!init_pipeline()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to restart pipeline");
  } else {
    RCLCPP_INFO(this->get_logger(), "Pipeline restarted successfully");
  }
}

}  // namespace bluerov2_controller
