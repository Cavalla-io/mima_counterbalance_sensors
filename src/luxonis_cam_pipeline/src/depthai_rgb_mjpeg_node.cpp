#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>  // for std::clamp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "depthai/depthai.hpp"

using namespace std::chrono_literals;

class DepthAIRgbMjpegNode : public rclcpp::Node {
public:
  DepthAIRgbMjpegNode() : Node("depthai_rgb_mjpeg_node") {
    // Same params as raw node (+ jpeg_quality)
    declare_parameter<std::string>("device_ip", "");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("mono_height", 720); // parity; unused
    declare_parameter<double>("fps", 30.0);
    declare_parameter<std::string>("topic_prefix", "oak");
    declare_parameter<int>("jpeg_quality", 90);  // 1..100
  }

  void init() {
    device_ip_     = get_parameter("device_ip").as_string();
    rgb_w_         = get_parameter("rgb_width").as_int();
    rgb_h_         = get_parameter("rgb_height").as_int();
    mono_h_        = get_parameter("mono_height").as_int(); (void)mono_h_;
    fps_           = get_parameter("fps").as_double();
    topic_prefix_  = get_parameter("topic_prefix").as_string();
    jpeg_quality_  = get_parameter("jpeg_quality").as_int();

    openDevice_();
    buildPipeline_();
    start_();
  }

private:
  // Unique-prefix resolver (checks base + /compressed)
  std::string chooseUniquePrefix_(const std::string& desired) {
    std::string ns = this->get_namespace();
    if(ns == "/") ns.clear();

    const std::vector<std::string> rels = {
      "rgb/image",
      "rgb/image/compressed"
    };

    auto abs_name = [&](const std::string& prefix, const std::string& rel){
      std::string s;
      if(!ns.empty()) s += ns;
      s += "/" + prefix + "/" + rel;
      return s;
    };

    std::set<std::string> existing;
    for(const auto& kv : this->get_topic_names_and_types()) {
      existing.insert(kv.first);
    }

    auto conflicts = [&](const std::string& prefix){
      for(const auto& r : rels) {
        if(existing.count(abs_name(prefix, r))) return true;
      }
      return false;
    };

    std::string candidate = desired;
    int i = 2;
    while(conflicts(candidate)) candidate = desired + "_" + std::to_string(i++);
    return candidate;
  }

  void openDevice_() {
    if(device_ip_.empty()) {
      device_ = std::make_unique<dai::Device>();
    } else {
      dai::DeviceInfo info(device_ip_); // IP or MXID
      device_ = std::make_unique<dai::Device>(info);
    }
    auto sockets = device_->getConnectedCameras();
    bool has_cam_a = false;
    for(auto s : sockets) if(s == dai::CameraBoardSocket::CAM_A) has_cam_a = true;
    if(!has_cam_a) {
      RCLCPP_FATAL(get_logger(), "Center RGB camera (CAM_A) not detected.");
      throw std::runtime_error("No CAM_A");
    }
    RCLCPP_INFO(get_logger(), "Found center camera: rgb");
  }

  void buildPipeline_() {
    pipeline_ = std::make_unique<dai::Pipeline>();

    // Color camera (CAM_A). Use ISP 'video' (NV12) path for encoder input.
    auto cam = pipeline_->create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setVideoSize(rgb_w_, rgb_h_);     // << size NV12 for encoder
    cam->setFps(fps_);
    // cam->video outputs NV12 by default; no ColorOrder/interleaved needed.

    // On-device MJPEG encoder (expects NV12/GRAY)
    auto enc = pipeline_->create<dai::node::VideoEncoder>();
    enc->setDefaultProfilePreset(fps_, dai::VideoEncoderProperties::Profile::MJPEG);
    enc->setQuality(std::clamp(jpeg_quality_, 1, 100));

    auto xout = pipeline_->create<dai::node::XLinkOut>();
    xout->setStreamName("xout_rgb_jpeg");

    cam->video.link(enc->input);      // << NV12 -> encoder
    enc->bitstream.link(xout->input);
  }

  void start_() {
    device_->startPipeline(*pipeline_);

    topic_prefix_ = chooseUniquePrefix_(topic_prefix_);
    RCLCPP_INFO(get_logger(), "Using topic prefix: %s", topic_prefix_.c_str());

    using Compressed = sensor_msgs::msg::CompressedImage;
    pub_ = this->create_publisher<Compressed>(
        topic_prefix_ + "/rgb/image/compressed", rclcpp::SensorDataQoS());

    // Encoded bitstream queue
    auto q = device_->getOutputQueue("xout_rgb_jpeg", /*maxSize*/4, /*blocking*/false);

    // Lightweight publishing thread; no decode on host
    thread_ = std::thread([this, q]() {
      rclcpp::WallRate idle(std::chrono::milliseconds(2));  // ~500 Hz idle loop
      while (rclcpp::ok()) {
        auto pkt = q->tryGet<dai::ImgFrame>(); // MJPEG bytes in getData()
        if (pkt) {
          Compressed msg;
          msg.header.stamp = this->now();
          msg.header.frame_id = "oak_rgb";
          msg.format = "jpeg";
          const auto& bytes = pkt->getData();
          msg.data.assign(bytes.begin(), bytes.end());
          pub_->publish(std::move(msg));
        } else {
          idle.sleep();
        }
      }
    });
    RCLCPP_INFO(get_logger(), "Publishing MJPEG -> %s",
                (topic_prefix_ + "/rgb/image/compressed").c_str());
  }

private:
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  std::thread thread_;

  // Params
  std::string device_ip_;
  int rgb_w_{1280}, rgb_h_{720}, mono_h_{720};
  double fps_{30.0};
  int jpeg_quality_{90};
  std::string topic_prefix_{"oak"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthAIRgbMjpegNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
