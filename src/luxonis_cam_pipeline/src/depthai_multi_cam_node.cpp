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

using namespace std::chrono_literals;

class DepthAIMultiCamNode : public rclcpp::Node {
public:
  DepthAIMultiCamNode() : Node("depthai_multi_cam_node") {
    declare_parameter<std::string>("device_ip", "");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<double>("fps", 30.0);
    declare_parameter<std::string>("topic_prefix", "oak");
  }

  void init() {
    device_ip_    = get_parameter("device_ip").as_string();
    rgb_w_        = get_parameter("rgb_width").as_int();
    rgb_h_        = get_parameter("rgb_height").as_int();
    fps_          = get_parameter("fps").as_double();
    topic_prefix_ = get_parameter("topic_prefix").as_string();

    openDevice_();
    buildPipeline_();
    start_();
  }

private:
  void openDevice_() {
    if(device_ip_.empty()) {
      device_ = std::make_unique<dai::Device>();
    } else {
      dai::DeviceInfo info(device_ip_);
      device_ = std::make_unique<dai::Device>(info);
    }
    RCLCPP_INFO(get_logger(), "Connected to OAK device");
  }

  void buildPipeline_() {
    pipeline_ = std::make_unique<dai::Pipeline>();

    auto cam = pipeline_->create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P); // hardcoded, should come from launch file later
    cam->setPreviewSize(rgb_w_, rgb_h_);
    cam->setFps(fps_);
    cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    cam->setInterleaved(true);

    auto xout = pipeline_->create<dai::node::XLinkOut>();
    xout->setStreamName("rgb");
    // Preview output is BGR interleaved and scaled
    cam->preview.link(xout->input);
  }

  void start_() {
    device_->startPipeline(*pipeline_);

    // Create ROS publisher
    // We only resolve the topic name once, simply
    std::string topic = topic_prefix_ + "/rgb/image";
    pub_ = create_publisher<sensor_msgs::msg::Image>(topic, 10);

    // Get output queue: 
    // maxSize=8 (buffer more frames to survive network jitter)
    // blocking=false (overwrite old frames if host is too slow)
    q_ = device_->getOutputQueue("rgb", 8, false);

    RCLCPP_INFO(get_logger(), "Streaming RGB to topic: %s", topic.c_str());

    // Simple polling thread
    thread_ = std::thread([this]() {
      while(rclcpp::ok()) {
        // Try to get a frame with a short timeout
        auto frame = q_->tryGet<dai::ImgFrame>();
        if(frame) {
            publishFrame_(*frame);
        } else {
            // Yield slightly to avoid 100% CPU spin
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  }

  void publishFrame_(const dai::ImgFrame& frame) {
    // Zero-copy (mostly) construction of ROS message
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "oak_rgb_frame";
    msg->height = frame.getHeight();
    msg->width = frame.getWidth();
    
    // "bgr8" because we set Interleaved(true) + BGR order
    msg->encoding = "bgr8"; 
    msg->step = frame.getWidth() * 3;
    
    // Copy data: vector assign
    const auto& data = frame.getData();
    msg->data.assign(data.begin(), data.end());

    pub_->publish(std::move(msg));
  }

  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::DataOutputQueue> q_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread thread_;

  // Params
  std::string device_ip_;
  int rgb_w_, rgb_h_;
  double fps_;
  std::string topic_prefix_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthAIMultiCamNode>();
  try {
      node->init();
      rclcpp::spin(node);
  } catch(const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
