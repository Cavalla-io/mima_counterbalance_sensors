#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

using namespace std::chrono_literals;

class DepthaiMultiCamNode : public rclcpp::Node {
public:
  DepthaiMultiCamNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
  : rclcpp::Node("depthai_multi_cam_node", opts)
  {
    declare_parameter<std::string>("device_ip", "");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("mono_height", 720); // parity; unused
    declare_parameter<double>("fps", 20.0);
    declare_parameter<std::string>("topic_prefix", "oak");
    declare_parameter<int>("idle_sleep_ms", 50);   // sleep when no subscribers
    declare_parameter<bool>("xlink_limit_fps", true); // clamp device->host fps at 'fps'
    declare_parameter<int>("frames_pool", 4);      // ImageManip pool size
  }

  ~DepthaiMultiCamNode() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

  void init() {
    device_ip_       = get_parameter("device_ip").as_string();
    rgb_w_           = get_parameter("rgb_width").as_int();
    rgb_h_           = get_parameter("rgb_height").as_int();
    mono_h_          = get_parameter("mono_height").as_int(); (void)mono_h_;
    fps_             = get_parameter("fps").as_double();
    topic_prefix_    = get_parameter("topic_prefix").as_string();
    idle_sleep_ms_   = get_parameter("idle_sleep_ms").as_int();
    xlink_limit_fps_ = get_parameter("xlink_limit_fps").as_bool();
    frames_pool_     = get_parameter("frames_pool").as_int();

    openDevice_();
    buildPipeline_();
    start_();
  }

private:
  std::string chooseUniquePrefix_(const std::string& desired) {
    std::string ns = this->get_namespace();
    if(ns == "/") ns.clear();
    const std::vector<std::string> rels = { "rgb/image" };

    auto abs_name = [&](const std::string& prefix, const std::string& rel){
      std::string s;
      if(!ns.empty()) s += ns;
      s += "/" + prefix + "/" + rel;
      return s;
    };

    std::set<std::string> existing;
    for(const auto& kv : this->get_topic_names_and_types()) existing.insert(kv.first);

    auto conflicts = [&](const std::string& prefix){
      for(const auto& r : rels) if(existing.count(abs_name(prefix, r))) return true;
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

    // Color camera: ISP NV12 at requested size/FPS
    auto cam = pipeline_->create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setVideoSize(rgb_w_, rgb_h_);
    cam->setFps(fps_);
    cam->setInterleaved(false);

    // On-device NV12 -> BGR8 (reduces host CPU)
    auto manip = pipeline_->create<dai::node::ImageManip>();
    manip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888i);
    manip->setNumFramesPool(std::max(2, frames_pool_));

    // Allow full-size BGR frames (no FOV loss)
    const std::size_t max_bytes = static_cast<std::size_t>(rgb_w_) * rgb_h_ * 3 + 4096; // BGR8 + padding
    manip->setMaxOutputFrameSize(max_bytes);

    auto xout = pipeline_->create<dai::node::XLinkOut>();
    xout->setStreamName("xout_rgb_bgr");
    if (xlink_limit_fps_) xout->setFpsLimit(fps_);

    cam->video.link(manip->inputImage);
    manip->out.link(xout->input);
  }

  void start_() {
    device_->startPipeline(*pipeline_);
    topic_prefix_ = chooseUniquePrefix_(topic_prefix_);
    RCLCPP_INFO(get_logger(), "Using topic prefix: %s", topic_prefix_.c_str());

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      topic_prefix_ + "/rgb/image",
      rclcpp::SensorDataQoS().keep_last(1)   // best-effort, depth 1
    );

    // Blocking queue; we’ll only call get() when needed
    auto q = device_->getOutputQueue("xout_rgb_bgr", /*maxSize*/1, /*blocking*/true);

    running_ = true;
    worker_ = std::thread([this, q]() {
      bool announced = false;
      const auto idle = std::chrono::milliseconds(std::max(0, idle_sleep_ms_));
      while (rclcpp::ok() && running_) {
        // Lazy gating: if nobody is subscribed, don't pull frames at all.
        if (pub_->get_subscription_count() == 0) {
          if (announced) {
            RCLCPP_INFO(this->get_logger(), "No subscribers; pausing frame pulls.");
            announced = false;
          }
          std::this_thread::sleep_for(idle);
          continue;
        }
        if (!announced) {
          RCLCPP_INFO(this->get_logger(), "Subscribers detected (%zu); pulling frames.",
                      pub_->get_subscription_count());
          announced = true;
        }

        auto frame = q->get<dai::ImgFrame>(); // BLOCKS until frame arrives
        if (!frame) continue;

        const int width  = frame->getWidth();
        const int height = frame->getHeight();
        const auto& data = frame->getData(); // BGR8 interleaved

        sensor_msgs::msg::Image msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "oak_rgb";
        msg.height = static_cast<uint32_t>(height);
        msg.width  = static_cast<uint32_t>(width);
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
        msg.step = static_cast<uint32_t>(width * 3);
        msg.data = data; // one copy into ROS message

        pub_->publish(std::move(msg));
      }
    });

    RCLCPP_INFO(get_logger(), "Publishing raw bgr8 -> %s",
                (topic_prefix_ + "/rgb/image").c_str());
  }

private:
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread worker_;
  std::atomic<bool> running_{false};

  // Params
  std::string device_ip_;
  int rgb_w_{1280}, rgb_h_{720}, mono_h_{720};
  double fps_{20.0};
  int idle_sleep_ms_{50};
  bool xlink_limit_fps_{true};
  int frames_pool_{4};
  std::string topic_prefix_{"oak"};
};

int main(int argc, char** argv) {
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthaiMultiCamNode>(opts);
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}