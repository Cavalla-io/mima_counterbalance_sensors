#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "depthai/depthai.hpp"

using namespace std::chrono_literals;

class DepthAIRgbH264Node : public rclcpp::Node {
public:
  DepthAIRgbH264Node() : Node("depthai_rgb_h264_node") {
    // Core params
    declare_parameter<std::string>("device_ip", "");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("mono_height", 720); // unused; parity
    declare_parameter<double>("fps", 30.0);
    declare_parameter<std::string>("topic_prefix", "oak");

    // Encoder params
    declare_parameter<int>("bitrate_kbps", 8000);       // 8 Mbps
    declare_parameter<int>("keyframe_interval", 60);    // IDR every N frames
    declare_parameter<bool>("high_profile", false);     // MAIN vs HIGH

    // Runtime behavior
    declare_parameter<bool>("xlink_limit_fps", true);   // clamp device->host to fps
    declare_parameter<int>("idle_sleep_ms", 50);        // sleep when no subs
  }

  ~DepthAIRgbH264Node() override {
    running_ = false;
    if(worker_.joinable()) worker_.join();
  }

  void init() {
    device_ip_         = get_parameter("device_ip").as_string();
    rgb_w_             = get_parameter("rgb_width").as_int();
    rgb_h_             = get_parameter("rgb_height").as_int();
    mono_h_            = get_parameter("mono_height").as_int(); (void)mono_h_;
    fps_               = get_parameter("fps").as_double();
    topic_prefix_      = get_parameter("topic_prefix").as_string();
    bitrate_kbps_      = get_parameter("bitrate_kbps").as_int();
    keyframe_interval_ = get_parameter("keyframe_interval").as_int();
    high_profile_      = get_parameter("high_profile").as_bool();
    xlink_limit_fps_   = get_parameter("xlink_limit_fps").as_bool();
    idle_sleep_ms_     = get_parameter("idle_sleep_ms").as_int();

    openDevice_();
    buildPipeline_();
    start_();
  }

private:
  std::string chooseUniquePrefix_(const std::string& desired) {
    std::string ns = this->get_namespace();
    if(ns == "/") ns.clear();
    const std::vector<std::string> rels = {
      "rgb/h264", "rgb/h265", "rgb/image", "rgb/image/compressed"
    };

    auto abs_name = [&](const std::string& prefix, const std::string& rel){
      std::string s; if(!ns.empty()) s += ns; s += "/" + prefix + "/" + rel; return s;
    };

    std::set<std::string> existing;
    for(const auto& kv : this->get_topic_names_and_types()) existing.insert(kv.first);

    auto conflicts = [&](const std::string& prefix){
      for(const auto& r : rels) if(existing.count(abs_name(prefix, r))) return true;
      return false;
    };

    std::string candidate = desired; int i = 2;
    while(conflicts(candidate)) candidate = desired + "_" + std::to_string(i++);
    return candidate;
  }

  void openDevice_() {
    if(device_ip_.empty()) device_ = std::make_unique<dai::Device>();
    else                   device_ = std::make_unique<dai::Device>(dai::DeviceInfo{device_ip_});

    auto sockets = device_->getConnectedCameras();
    bool has_cam_a = false;
    for(auto s : sockets) if(s == dai::CameraBoardSocket::CAM_A) { has_cam_a = true; break; }
    if(!has_cam_a) {
      RCLCPP_FATAL(get_logger(), "Center RGB camera (CAM_A) not detected.");
      throw std::runtime_error("No CAM_A");
    }
    RCLCPP_INFO(get_logger(), "Found center camera: rgb");
  }

  void buildPipeline_() {
    pipeline_ = std::make_unique<dai::Pipeline>();

    // Camera → NV12
    auto cam = pipeline_->create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setVideoSize(rgb_w_, rgb_h_);
    cam->setFps(fps_);
    cam->setInterleaved(false); // NV12 planes

    // Encoder → H.264 bitstream
    auto enc = pipeline_->create<dai::node::VideoEncoder>();
    enc->setDefaultProfilePreset(
      fps_,
      high_profile_ ? dai::VideoEncoderProperties::Profile::H264_HIGH
                    : dai::VideoEncoderProperties::Profile::H264_MAIN
    );
    enc->setBitrate(std::max(1, bitrate_kbps_) * 1000);    // kbps → bps
    enc->setKeyframeFrequency(std::max(1, keyframe_interval_));
    enc->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
    enc->setNumBFrames(0);              // lower decode latency
    // enc->setInsertSpsPps(true);      // not available in your DepthAI build

    auto xout = pipeline_->create<dai::node::XLinkOut>();
    xout->setStreamName("xout_rgb_h264");
    if (xlink_limit_fps_) xout->setFpsLimit(fps_);

    cam->video.link(enc->input);
    enc->bitstream.link(xout->input);
  }

  void start_() {
    device_->startPipeline(*pipeline_);

    topic_prefix_ = chooseUniquePrefix_(topic_prefix_);
    RCLCPP_INFO(get_logger(), "Using topic prefix: %s", topic_prefix_.c_str());

    using Compressed = sensor_msgs::msg::CompressedImage;
    auto qos = rclcpp::SensorDataQoS().keep_last(1);  // latest-only
    pub_ = this->create_publisher<Compressed>(topic_prefix_ + "/rgb/h264", qos);

    // Device queue: depth=1, non-blocking (drop-old on device). Host uses blocking get().
    auto q = device_->getOutputQueue("xout_rgb_h264", /*maxSize*/1, /*blocking*/false);

    running_ = true;
    worker_ = std::thread([this, q]() {
      const auto idle = std::chrono::milliseconds(std::max(0, idle_sleep_ms_));
      bool announced = false;

      while (rclcpp::ok() && running_) {
        // Lazy: don't pull encoded data if nobody is listening
        if (pub_->get_subscription_count() == 0) {
          if (announced) {
            RCLCPP_INFO(this->get_logger(), "No subscribers; pausing encoded pulls.");
            announced = false;
          }
          std::this_thread::sleep_for(idle);
          continue;
        }
        if (!announced) {
          RCLCPP_INFO(this->get_logger(), "Subscribers detected (%zu); pulling encoded frames.",
                      pub_->get_subscription_count());
          announced = true;
        }

        // Block for at least one packet
        auto pkt = q->get<dai::ImgFrame>();
        if(!pkt) continue;

        // Drain backlog so we only publish the newest packet
        while (auto newer = q->tryGet<dai::ImgFrame>()) {
          pkt = std::move(newer);
        }

        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "oak_rgb";
        msg.format = "h264";

        const auto& bytes = pkt->getData();
        msg.data.assign(bytes.begin(), bytes.end());
        pub_->publish(std::move(msg));
      }
    });

    RCLCPP_INFO(get_logger(), "Publishing H.264 -> %s",
                (topic_prefix_ + "/rgb/h264").c_str());
  }

private:
  // State
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  std::thread worker_;
  std::atomic<bool> running_{false};

  // Params
  std::string device_ip_;
  int rgb_w_{1280}, rgb_h_{720}, mono_h_{720};
  double fps_{30.0};
  int bitrate_kbps_{8000};
  int keyframe_interval_{60};
  bool high_profile_{false};
  bool xlink_limit_fps_{true};
  int idle_sleep_ms_{50};
  std::string topic_prefix_{"oak"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthAIRgbH264Node>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
