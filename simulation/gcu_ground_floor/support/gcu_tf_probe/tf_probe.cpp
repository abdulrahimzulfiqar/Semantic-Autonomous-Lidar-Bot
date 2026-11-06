// Read-only audit. GIDs come from rclcpp MessageInfo, unavailable in this
// installation's older rclpy callback and take-message metadata interfaces.
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>

class Probe : public rclcpp::Node {
public:
  Probe() : Node("gcu_tf_ownership_probe")
  {
    owners_["map->odom"] = {};
    owners_["odom->base_footprint"] = {};
    owners_["AGmap->map"] = {};
    owners_["gcu_probe_parent->gcu_probe_child"] = {};
    auto callback = [this](const tf2_msgs::msg::TFMessage::SharedPtr message,
        const rclcpp::MessageInfo & info) {
        std::ostringstream identity;
        for (auto byte : info.get_rmw_message_info().publisher_gid.data) {
          identity << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
        }
        for (const auto & transform : message->transforms) {
          const auto edge = transform.header.frame_id + "->" + transform.child_frame_id;
          if (owners_.count(edge)) {owners_[edge].insert(identity.str());}
        }
      };
    dynamic_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SensorDataQoS(), callback);
    static_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(100).transient_local(), callback);
    output_ = create_publisher<std_msgs::msg::String>("/gcu/tf_ownership_report", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
        std::ostringstream json;
        json << "{\"source\":\"rclcpp_publisher_gid\",\"edge_publishers\":{";
        bool first = true;
        for (const auto & entry : owners_) {
          if (!first) {json << ",";} first = false;
          json << std::quoted(entry.first) << ":" << entry.second.size();
        }
        json << "}}";
        std_msgs::msg::String message; message.data = json.str(); output_->publish(message);
      });
  }
private:
  std::map<std::string, std::set<std::string>> owners_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_, static_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Probe>());
  rclcpp::shutdown();
  return 0;
}
