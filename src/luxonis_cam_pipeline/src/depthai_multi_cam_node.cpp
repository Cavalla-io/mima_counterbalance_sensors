#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
    std::string cam_info_topic = topic_prefix_ + "/rgb/camera_info";
    pub_cam_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic, 10);

    // Get camera intrinsics
    auto calibData = device_->readCalibration();
    auto intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, rgb_w_, rgb_h_);
    cam_info_msg_.width = rgb_w_;
    cam_info_msg_.height = rgb_h_;
    cam_info_msg_.k[0] = intrinsics[0][0];
    cam_info_msg_.k[1] = 0.0;
    cam_info_msg_.k[2] = intrinsics[0][2];
    cam_info_msg_.k[3] = 0.0;
    cam_info_msg_.k[4] = intrinsics[1][1];
    cam_info_msg_.k[5] = intrinsics[1][2];
    cam_info_msg_.k[6] = 0.0;
    cam_info_msg_.k[7] = 0.0;
    cam_info_msg_.k[8] = 1.0;
    cam_info_msg_.p[0] = cam_info_msg_.k[0];
    cam_info_msg_.p[1] = 0.0;
    cam_info_msg_.p[2] = cam_info_msg_.k[2];
    cam_info_msg_.p[3] = 0.0;
    cam_info_msg_.p[4] = 0.0;
    cam_info_msg_.p[5] = cam_info_msg_.k[4];
    cam_info_msg_.p[6] = cam_info_msg_.k[5];
    cam_info_msg_.p[7] = 0.0;
    cam_info_msg_.p[8] = 0.0;
    cam_info_msg_.p[9] = 0.0;
    cam_info_msg_.p[10] = 1.0;
    cam_info_msg_.p[11] = 0.0;


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
    
    auto now = this->now();
    msg->header.stamp = now;
    msg->header.frame_id = "oak_rgb_frame";
    msg->height = frame.getHeight();
    msg->width = frame.getWidth();
    
    // "bgr8" because we set Interleaved(true) + BGR order
    msg->encoding = "bgr8"; 
    msg->step = frame.getWidth() * 3;
    
    // Copy data: vector assign
    const auto& data = frame.getData();
    msg->data.assign(data.begin(), data.end());

    cam_info_msg_.header.stamp = now;
    cam_info_msg_.header.frame_id = "oak_rgb_frame";
    pub_cam_info_->publish(cam_info_msg_);

    pub_->publish(std::move(msg));
  }

  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::DataOutputQueue> q_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;
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
