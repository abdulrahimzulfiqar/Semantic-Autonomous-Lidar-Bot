/**
 * @file osmag_planner_node.cpp
 * @brief ROS 2 Humble C++ Node for osmAG Semantic Hierarchical Global Path Planning & Real-Time Path Execution
 *
 * This node is the brain of the autonomous wheelchair navigation system. It:
 *   1. Loads a pre-built osmAG topometric map (.osm) describing rooms and doorways
 *   2. Publishes 3D visualization markers for Foxglove Studio / RViz2
 *   3. Listens for /goal_pose clicks from the user
 *   4. Plans a multi-room route through doorways using A* on the Area Graph
 *   5. Drives the robot along the planned route using a Pure Pursuit velocity controller
 *
 * Subscribes:
 *   - /goal_pose (geometry_msgs/msg/PoseStamped): Target navigation goal from Foxglove Studio / RViz2
 *
 * Publishes:
 *   - /osmag/map_markers (visualization_msgs/msg/MarkerArray): 3D area polygons & doorway lines
 *   - /osmag/global_path (nav_msgs/msg/Path): Hierarchical planned trajectory
 *   - /osmag/path_marker (visualization_msgs/msg/Marker): 3D thick route line for Foxglove Studio
 *   - /cmd_vel (geometry_msgs/msg/Twist): Velocity commands for twist_mux → diff_controller → wheels
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include "data_load_save.h"
#include "pathgraph.h"
#include "area_grid_map.h"
#include "visualization.h"

using namespace osm_ag;

class OsmAgPlannerNode : public rclcpp::Node {
public:
    OsmAgPlannerNode() : Node("osmag_planner_node"), current_waypoint_idx_(0), is_navigating_(false), log_counter_(0) {
        // === Declare parameters (all configurable via YAML or command line) ===
        this->declare_parameter<std::string>("osm_file_path", "data/big_map_7.osm");
        this->declare_parameter<std::string>("map_frame", "odom");     // Use odom in sim (no AMCL map→odom initially)
        this->declare_parameter<std::string>("base_frame", "base_footprint"); // Must match EKF/AMCL base_frame_id
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("max_linear_speed", 0.30);
        this->declare_parameter<double>("max_angular_speed", 0.7);
        this->declare_parameter<double>("lookahead_dist", 0.35);
        this->declare_parameter<double>("goal_tolerance", 0.20);

        this->get_parameter("osm_file_path", osm_file_path_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("max_linear_speed", max_linear_speed_);
        this->get_parameter("max_angular_speed", max_angular_speed_);
        this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("goal_tolerance", goal_tolerance_);

        if (osm_file_path_.empty() || osm_file_path_[0] != '/') {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("lidarbot_osmag");
            osm_file_path_ = pkg_share + "/" + osm_file_path_;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing osmAG Planner & Path Follower Node...");
        RCLCPP_INFO(this->get_logger(), "  OSM File: %s", osm_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  TF Frames: %s -> %s", map_frame_.c_str(), base_frame_.c_str());

        // === Parse OSM-AG File ===
        if (!Init_OSMAG(graph_, osm_file_path_.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", osm_file_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Parsed %zu areas and %zu passages successfully!",
                    graph_.areas_.size(), graph_.passages_.size());

        // === Pre-compute 2D grid submaps ===
        RCLCPP_INFO(this->get_logger(), "Building 2D Grid Submaps (resolution: %.3f m)...", resolution_);
        for (auto& pair : graph_.areas_) {
            InitOccupancyMap_Area(graph_, pair.first, resolution_);
        }
        RCLCPP_INFO(this->get_logger(), "2D Grid Submaps initialization complete!");

        // === TF2 Listener ===
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // === Publishers ===
        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
        map_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/osmag/map_markers", qos_transient);
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/osmag/global_path", rclcpp::QoS(10).reliable());
        path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/osmag/path_marker", rclcpp::QoS(10).reliable());

        // Publish to /cmd_vel — twist_mux remaps this to /diff_controller/cmd_vel_unstamped
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // === Subscriber ===
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&OsmAgPlannerNode::GoalCallback, this, std::placeholders::_1));

        // === Timers ===
        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&OsmAgPlannerNode::PublishMapMarkers, this));

        // Path-Following Control Loop at 20 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OsmAgPlannerNode::ControlLoop, this));

        PublishMapMarkers();

        RCLCPP_INFO(this->get_logger(), "osmAG Planner & Controller Node ready! Click /goal_pose in Foxglove to navigate.");
    }

private:
    // ========================================================================
    //  GOAL CALLBACK — called when user clicks a goal pose in Foxglove/RViz2
    // ========================================================================
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(), "=== Received Navigation Goal Pose: (x=%.2f, y=%.2f) ===", goal_x, goal_y);

        if (graph_.passages_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No passages available in graph!");
            return;
        }

        // 1. Get current robot pose
        double start_x = 0.0, start_y = 0.0;
        GetRobotPose(start_x, start_y);

        RCLCPP_INFO(this->get_logger(), "Robot Start Pose: (x=%.2f, y=%.2f)", start_x, start_y);

        // 2. Find closest passage for start and goal
        PassageId start_id = FindClosestPassage(start_x, start_y);
        PassageId goal_id = FindClosestPassage(goal_x, goal_y);

        std::vector<Eigen::Vector3d> path_result;

        if (start_id == goal_id) {
            RCLCPP_INFO(this->get_logger(), "Start and goal in same room (Passage %ld). Direct path.", start_id);
            path_result.push_back(Eigen::Vector3d(start_x, start_y, 0.0));
            path_result.push_back(Eigen::Vector3d(goal_x, goal_y, 0.0));
        } else {
            RCLCPP_INFO(this->get_logger(), "Planning topometric route: Passage %ld -> Passage %ld...", start_id, goal_id);

            auto t_start = std::chrono::high_resolution_clock::now();
            PlanInPathGraph(graph_, start_id, goal_id, path_result);
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

            if (path_result.empty()) {
                RCLCPP_WARN(this->get_logger(), "No valid route between Passage %ld and %ld.", start_id, goal_id);
                return;
            }

            // Prepend actual start and append actual goal to the A* waypoints
            path_result.insert(path_result.begin(), Eigen::Vector3d(start_x, start_y, 0.0));
            path_result.push_back(Eigen::Vector3d(goal_x, goal_y, 0.0));

            RCLCPP_INFO(this->get_logger(), "Plan completed in %.3f ms! Generated %zu waypoints.", elapsed_ms, path_result.size());
        }

        // Publish visualization
        PublishPath(path_result);

        // Activate the velocity controller
        active_waypoints_ = path_result;
        current_waypoint_idx_ = 0;
        is_navigating_ = true;
        log_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), ">>> Path Following STARTED with %zu waypoints <<<", active_waypoints_.size());
    }

    // ========================================================================
    //  CONTROL LOOP — runs at 20 Hz, drives the robot along the active path
    // ========================================================================
    void ControlLoop() {
        if (!is_navigating_ || active_waypoints_.empty()) {
            return;
        }

        // 1. Get current robot pose
        double rx = 0.0, ry = 0.0, ryaw = 0.0;
        if (!GetRobotPoseWithYaw(rx, ry, ryaw)) {
            // Log once every 2 seconds (40 iterations at 20Hz) to avoid spam
            if (log_counter_ % 40 == 0) {
                RCLCPP_WARN(this->get_logger(), "ControlLoop: Waiting for TF (%s -> %s)...", map_frame_.c_str(), base_frame_.c_str());
            }
            log_counter_++;
            return;
        }

        // 2. Advance waypoint index — skip waypoints that are already behind us
        while (current_waypoint_idx_ < active_waypoints_.size()) {
            double wx = active_waypoints_[current_waypoint_idx_][0];
            double wy = active_waypoints_[current_waypoint_idx_][1];
            double dist = std::hypot(wx - rx, wy - ry);
            if (dist < lookahead_dist_ && current_waypoint_idx_ + 1 < active_waypoints_.size()) {
                current_waypoint_idx_++;
            } else {
                break;
            }
        }

        // 3. Check if we reached the final goal
        double goal_x = active_waypoints_.back()[0];
        double goal_y = active_waypoints_.back()[1];
        double dist_to_goal = std::hypot(goal_x - rx, goal_y - ry);

        if (dist_to_goal < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "*** GOAL REACHED (%.2f m away) ***", dist_to_goal);
            StopRobot();
            is_navigating_ = false;
            active_waypoints_.clear();
            return;
        }

        // 4. Pure Pursuit — compute velocity command towards current target waypoint
        double tx = active_waypoints_[current_waypoint_idx_][0];
        double ty = active_waypoints_[current_waypoint_idx_][1];
        double dist_to_wp = std::hypot(tx - rx, ty - ry);

        double target_heading = std::atan2(ty - ry, tx - rx);
        double heading_err = target_heading - ryaw;

        // Normalize to [-pi, pi]
        while (heading_err > M_PI) heading_err -= 2.0 * M_PI;
        while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        if (std::abs(heading_err) > 0.6) {
            // Large heading error: rotate in place first
            cmd.linear.x = 0.0;
            cmd.angular.z = std::clamp(1.2 * heading_err, -max_angular_speed_, max_angular_speed_);
        } else {
            // Drive forward while steering
            double speed_scale = std::max(0.3, std::cos(heading_err));
            cmd.linear.x = std::min(max_linear_speed_ * speed_scale, dist_to_goal * 0.5);
            cmd.linear.x = std::max(cmd.linear.x, 0.05); // minimum forward motion
            cmd.angular.z = std::clamp(1.5 * heading_err, -max_angular_speed_, max_angular_speed_);
        }

        cmd_vel_pub_->publish(cmd);

        // Log progress every 1 second (20 iterations)
        if (log_counter_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Nav: wp=%zu/%zu  pos=(%.2f,%.2f)  yaw=%.1f°  goal_dist=%.2f  v=%.2f  w=%.2f",
                current_waypoint_idx_, active_waypoints_.size(),
                rx, ry, ryaw * 180.0 / M_PI,
                dist_to_goal,
                cmd.linear.x, cmd.angular.z);
        }
        log_counter_++;
    }

    // ========================================================================
    //  TF HELPERS — get the robot pose from the TF tree
    // ========================================================================
    bool GetRobotPoseWithYaw(double &x, double &y, double &yaw) {
        try {
            std::string tf_err;
            geometry_msgs::msg::TransformStamped tf_stamped;

            // Try primary frame (odom in sim, map on real robot with AMCL)
            if (tf_buffer_->canTransform(map_frame_, base_frame_, tf2::TimePointZero, &tf_err)) {
                tf_stamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            }
            // Fallback: try odom if primary is map and map isn't available yet
            else if (map_frame_ != "odom" && tf_buffer_->canTransform("odom", base_frame_, tf2::TimePointZero, &tf_err)) {
                tf_stamped = tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);
            }
            else {
                return false;
            }

            x = tf_stamped.transform.translation.x;
            y = tf_stamped.transform.translation.y;

            tf2::Quaternion q(
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w);
            yaw = tf2::getYaw(q);
            return true;
        } catch (const tf2::TransformException &ex) {
            return false;
        }
    }

    void GetRobotPose(double &x, double &y) {
        double yaw;
        if (!GetRobotPoseWithYaw(x, y, yaw)) {
            x = 0.0;
            y = 0.0;
            RCLCPP_WARN(this->get_logger(), "TF unavailable for start pose. Using (0,0) fallback.");
        }
    }

    void StopRobot() {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        // Publish stop multiple times to ensure it gets through
        for (int i = 0; i < 5; ++i) {
            cmd_vel_pub_->publish(stop_cmd);
        }
    }

    // ========================================================================
    //  GRAPH HELPERS
    // ========================================================================
    PassageId FindClosestPassage(double x, double y) {
        PassageId best_id = graph_.passages_.begin()->first;
        double min_dist = 1e9;
        for (auto& pair : graph_.passages_) {
            double px = pair.second->center_position[0];
            double py = pair.second->center_position[1];
            double dist = std::hypot(px - x, py - y);
            if (dist < min_dist) {
                min_dist = dist;
                best_id = pair.first;
            }
        }
        return best_id;
    }

    // ========================================================================
    //  VISUALIZATION
    // ========================================================================
    void PublishMapMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        // Area Polygons (Green Lines)
        for (auto& pair : graph_.areas_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // Always use odom for static markers
            marker.header.stamp = this->now();
            marker.ns = "areas";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.08;
            marker.color.r = 0.0f;
            marker.color.g = 0.85f;
            marker.color.b = 0.25f;
            marker.color.a = 0.9f;

            for (auto node_id : pair.second->nodes_inorder_) {
                if (graph_.nodes_.find(node_id) != graph_.nodes_.end()) {
                    geometry_msgs::msg::Point pt;
                    pt.x = graph_.nodes_[node_id]->attributes_->position[0];
                    pt.y = graph_.nodes_[node_id]->attributes_->position[1];
                    pt.z = 0.0;
                    marker.points.push_back(pt);
                }
            }
            if (!marker.points.empty()) {
                marker.points.push_back(marker.points.front());
                marker_array.markers.push_back(marker);
            }
        }

        // Passage Doorways (Red Lines)
        for (auto& pair : graph_.passages_) {
            auto src_id = pair.second->passage_nodes.source;
            auto tgt_id = pair.second->passage_nodes.target;
            if (graph_.nodes_.find(src_id) != graph_.nodes_.end() && graph_.nodes_.find(tgt_id) != graph_.nodes_.end()) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = this->now();
                marker.ns = "passages";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.15;
                marker.color.r = 1.0f;
                marker.color.g = 0.1f;
                marker.color.b = 0.1f;
                marker.color.a = 1.0f;

                geometry_msgs::msg::Point p1, p2;
                p1.x = graph_.nodes_[src_id]->attributes_->position[0];
                p1.y = graph_.nodes_[src_id]->attributes_->position[1];
                p1.z = 0.05;
                p2.x = graph_.nodes_[tgt_id]->attributes_->position[0];
                p2.y = graph_.nodes_[tgt_id]->attributes_->position[1];
                p2.z = 0.05;

                marker.points.push_back(p1);
                marker.points.push_back(p2);
                marker_array.markers.push_back(marker);
            }
        }

        map_markers_pub_->publish(marker_array);
    }

    void PublishPath(const std::vector<Eigen::Vector3d>& waypoints) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = this->now();

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "odom";
        line_marker.header.stamp = this->now();
        line_marker.ns = "planned_route";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.15;
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.0f;
        line_marker.color.b = 1.0f;
        line_marker.color.a = 1.0f;

        for (const auto& pt : waypoints) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.header.stamp = this->now();
            pose.pose.position.x = pt[0];
            pose.pose.position.y = pt[1];
            pose.pose.position.z = pt[2];
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            geometry_msgs::msg::Point marker_pt;
            marker_pt.x = pt[0];
            marker_pt.y = pt[1];
            marker_pt.z = pt[2] + 0.05;
            line_marker.points.push_back(marker_pt);
        }

        global_path_pub_->publish(path_msg);
        path_marker_pub_->publish(line_marker);
        RCLCPP_INFO(this->get_logger(), "Published global path with %zu poses to /osmag/global_path", path_msg.poses.size());
    }

    // ========================================================================
    //  MEMBER VARIABLES
    // ========================================================================
    std::string osm_file_path_;
    std::string map_frame_;
    std::string base_frame_;
    double resolution_;
    double max_linear_speed_;
    double max_angular_speed_;
    double lookahead_dist_;
    double goal_tolerance_;

    AreaGraph graph_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<Eigen::Vector3d> active_waypoints_;
    size_t current_waypoint_idx_;
    bool is_navigating_;
    int log_counter_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OsmAgPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
