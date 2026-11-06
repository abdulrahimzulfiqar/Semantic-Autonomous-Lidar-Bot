// GCU config-file adapter around the installed ros_gz_bridge conversions.
// Destroy transport subscriptions while the ROS context is still valid.
#include <csignal>
#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace { volatile std::sig_atomic_t stopping = 0;
void request_stop(int) {stopping = 1;} }

class OrderedBridge : public ros_gz_bridge::RosGzBridge {
public:
  void stop_transport()
  {
    heartbeat_timer_->cancel();
    for (const auto & topic : gz_node_->SubscribedTopics()) {gz_node_->Unsubscribe(topic);}
    handles_.clear();
    services_.clear();
    gz_node_.reset();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto bridge = std::make_shared<OrderedBridge>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  while (!stopping && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor.remove_node(bridge);
  bridge->stop_transport();
  bridge.reset();
  rclcpp::shutdown();
  return 0;
}
