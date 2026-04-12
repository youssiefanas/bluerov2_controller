#ifndef BLUEROV2_CONTROLLER__CAMERA_STREAMER_NODE_HPP_
#define BLUEROV2_CONTROLLER__CAMERA_STREAMER_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace bluerov2_controller
{

/**
 * @brief ROS 2 node for receiving and publishing the BlueROV2 H.264 camera stream.
 *
 * Uses the GStreamer C API to receive an RTP/H.264 UDP video stream from BlueOS,
 * decode it (with hardware acceleration when available), and publish frames as
 * sensor_msgs::msg::Image via image_transport. Includes a watchdog that
 * automatically restarts the pipeline on stream loss.
 */
class CameraStreamerNode : public rclcpp::Node
{
public:
  explicit CameraStreamerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraStreamerNode() override;

private:
  /**
   * @brief Initializes the GStreamer pipeline and starts video capture.
   * @return True if the pipeline was created and set to PLAYING successfully.
   */
  bool init_pipeline();

  /**
   * @brief Stops and releases all GStreamer pipeline resources.
   */
  void shutdown_pipeline();

  /**
   * @brief Constructs the GStreamer pipeline description string based on parameters.
   * @return Pipeline string suitable for gst_parse_launch().
   */
  std::string build_pipeline_string() const;

  /**
   * @brief Selects the best available H.264 decoder.
   *
   * When the decoder parameter is "auto", tries hardware decoders (vaapih264dec,
   * nvh264dec) first via gst_element_factory_find(), falling back to software avdec_h264.
   * @return GStreamer element name for the selected decoder.
   */
  std::string select_decoder() const;

  /**
   * @brief GStreamer appsink callback invoked on the streaming thread when a new frame arrives.
   * @param sink The GStreamer appsink element.
   * @param user_data Pointer to the CameraStreamerNode instance.
   * @return GST_FLOW_OK on success, GST_FLOW_ERROR if the sample could not be pulled.
   */
  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data);

  /**
   * @brief Extracts frame data from a GstSample and publishes it as a ROS Image message.
   *
   * Maps the GStreamer buffer directly into sensor_msgs::msg::Image data to avoid
   * unnecessary copies. If resize is enabled, uses OpenCV to rescale the frame.
   * @param sample GStreamer sample containing the decoded video frame.
   */
  void publish_frame(GstSample * sample);

  /**
   * @brief Periodic watchdog callback that checks for incoming frames and restarts the pipeline if stalled.
   */
  void watchdog_callback();

  /**
   * @brief Shuts down and reinitializes the GStreamer pipeline after a stream loss.
   */
  void restart_pipeline();

  // GStreamer resources
  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};

  /// Camera image publisher via image_transport (auto-generates /compressed and /theora variants)
  image_transport::Publisher image_pub_;

  // Watchdog timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::atomic<uint64_t> frame_count_{0};
  uint64_t last_watchdog_count_{0};
  int watchdog_failures_{0};

  // Parameters
  int port_;
  std::string decoder_;
  bool resize_enable_;
  int resize_width_;
  int resize_height_;
  std::string frame_id_;
  double watchdog_period_sec_;
  int max_watchdog_failures_;
};

}  // namespace bluerov2_controller

#endif  // BLUEROV2_CONTROLLER__CAMERA_STREAMER_NODE_HPP_
