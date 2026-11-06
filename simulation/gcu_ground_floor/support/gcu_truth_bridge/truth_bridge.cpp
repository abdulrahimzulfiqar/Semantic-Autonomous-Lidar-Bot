// Evaluation only: Gazebo model poses never enter /tf or navigation inputs.
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace {
volatile std::sig_atomic_t requested_stop = 0;
void stop_signal(int) {requested_stop = 1;}

struct Bridge {
  std::mutex mutex;
  bool stopping = false;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher;
  std::string model;
  std::size_t count = 0;

  void receive(const ignition::msgs::Pose_V & input)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (stopping) {return;}
    tf2_msgs::msg::TFMessage output;
    for (const auto & pose : input.pose()) {
      const auto & name = pose.name();
      if (name != model && (name.size() < model.size() + 2 ||
        name.substr(name.size() - model.size() - 2) != "::" + model)) {continue;}
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "world";
      // PosePublisher stamps each pose; the outer Pose_V header can be empty.
      transform.header.stamp.sec = pose.header().stamp().sec();
      transform.header.stamp.nanosec = pose.header().stamp().nsec();
      transform.child_frame_id = model;
      transform.transform.translation.x = pose.position().x();
      transform.transform.translation.y = pose.position().y();
      transform.transform.translation.z = pose.position().z();
      transform.transform.rotation.x = pose.orientation().x();
      transform.transform.rotation.y = pose.orientation().y();
      transform.transform.rotation.z = pose.orientation().z();
      transform.transform.rotation.w = pose.orientation().w();
      output.transforms.push_back(transform);
    }
    if (!output.transforms.empty()) {publisher->publish(output); ++count;}
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  // Keep the ROS context alive until transport callbacks have been stopped.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
  auto node = std::make_shared<rclcpp::Node>("gcu_truth_bridge");
  const auto topic = node->declare_parameter<std::string>("source_topic", "/model/lidarbot/pose");
  const auto output = node->declare_parameter<std::string>("output_topic", "/gcu_ground_truth");
  Bridge bridge;
  bridge.model = node->declare_parameter<std::string>("model", "lidarbot");
  bridge.publisher = node->create_publisher<tf2_msgs::msg::TFMessage>(output, rclcpp::SensorDataQoS());
  {
    ignition::transport::Node transport;
    if (!transport.Subscribe(topic, &Bridge::receive, &bridge)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to subscribe to %s", topic.c_str());
      rclcpp::shutdown();
      return 1;
    }
    while (!requested_stop && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    {
      std::lock_guard<std::mutex> lock(bridge.mutex);
      bridge.stopping = true;
    }
    transport.Unsubscribe(topic);
  }
  RCLCPP_INFO(node->get_logger(), "Transport callbacks stopped before ROS shutdown; %zu pose messages", bridge.count);
  bridge.publisher.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
