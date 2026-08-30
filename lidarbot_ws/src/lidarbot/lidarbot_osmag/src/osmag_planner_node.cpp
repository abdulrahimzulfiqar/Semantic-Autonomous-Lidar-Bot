/**
 * @file osmag_planner_node.cpp
 * @brief ROS 2 Humble C++ Node for osmAG Semantic Hierarchical Global Path Planning
 *        with Nav2 obstacle-avoiding path execution.
 *
 * == Architecture ==
 * This node has TWO jobs:
 *   1. GLOBAL PLANNING — Uses the osmAG topometric map to compute a route through
 *      rooms and doorways (A* on the Area Graph). This is the "which rooms do I
 *      traverse?" decision.
 *   2. DELEGATED EXECUTION — Sends the planned global path to Nav2's controller_server
 *      via the FollowPath action. Nav2's RegulatedPurePursuitController then drives
 *      the robot along the path while using its local_costmap (fed by LiDAR /scan)
 *      to avoid obstacles like furniture, people, and other dynamic objects.
 *
 * == Why Not Drive Directly? ==
 * The previous version had its own Pure Pursuit controller that published /cmd_vel
 * directly. This worked but was BLIND — it couldn't see obstacles. By delegating
 * to Nav2's controller_server, we get:
 *   - Local costmap with obstacle inflation from LiDAR
 *   - Collision detection and emergency stopping
 *   - Velocity scaling near obstacles
 *   - Progress checking (detect if robot is stuck)
 *
 * == Data Flow ==
 *   User clicks goal → osmAG plans global route → sends FollowPath action →
 *   Nav2 controller_server follows path with obstacle avoidance →
 *   /cmd_vel → twist_mux → diff_controller → wheels
 *
 * Subscribes:
 *   - /goal_pose (geometry_msgs/msg/PoseStamped): Target goal from Foxglove/RViz2
 *
 * Publishes:
 *   - /osmag/map_markers (visualization_msgs/msg/MarkerArray): 3D area polygons & doorways
 *   - /osmag/global_path (nav_msgs/msg/Path): Planned trajectory for visualization
 *   - /osmag/path_marker (visualization_msgs/msg/Marker): Thick magenta route line
 *   - /osmag/status (std_msgs/msg/String): Human-readable navigation status
 *
 * Action Clients:
 *   - /follow_path (nav2_msgs/action/FollowPath): Sends path to Nav2 controller
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "data_load_save.h"
#include "pathgraph.h"
#include "area_grid_map.h"
#include "visualization.h"

using namespace osm_ag;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class OsmAgPlannerNode : public rclcpp::Node {
public:
    OsmAgPlannerNode() : Node("osmag_planner_node"), is_navigating_(false) {
        // === Declare parameters (all configurable via YAML or command line) ===
        this->declare_parameter<std::string>("osm_file_path", "data/big_map_7.osm");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<double>("resolution", 0.05);

        this->get_parameter("osm_file_path", osm_file_path_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("resolution", resolution_);

        if (osm_file_path_.empty() || osm_file_path_[0] != '/') {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("lidarbot_osmag");
            osm_file_path_ = pkg_share + "/" + osm_file_path_;
        }

        RCLCPP_INFO(this->get_logger(), "=== Initializing osmAG Global Planner + Nav2 Bridge ===");
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
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/osmag/status", rclcpp::QoS(10).reliable());

        // === Nav2 FollowPath Action Client ===
        // This connects to the controller_server's FollowPath action, which uses
        // the local costmap (with LiDAR obstacle data) to follow our path safely.
        follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        // === Subscriber for goal pose ===
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&OsmAgPlannerNode::GoalCallback, this, std::placeholders::_1));

        // === Periodic timer to publish map markers (every 2 seconds) ===
        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&OsmAgPlannerNode::PublishMapMarkers, this));

        // === Status monitoring timer (every 1 second) ===
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OsmAgPlannerNode::StatusCheck, this));

        // Initial publish of campus map 3D markers
        PublishMapMarkers();

        RCLCPP_INFO(this->get_logger(), "osmAG Global Planner Node ready! Click /goal_pose in Foxglove to navigate.");
        RCLCPP_INFO(this->get_logger(), "Nav2 controller_server will handle obstacle avoidance via local_costmap + LiDAR.");
    }

private:
    // ========================================================================
    //  GOAL CALLBACK — called when user clicks a goal pose in Foxglove/RViz2
    // ========================================================================
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        // Automatically transform goal coordinates if sent in a different frame (e.g. map vs odom)
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
            try {
                std::string tf_err;
                if (tf_buffer_->canTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero, &tf_err)) {
                    geometry_msgs::msg::PoseStamped transformed_msg;
                    tf_stamped_transform(msg, transformed_msg, map_frame_);
                    goal_x = transformed_msg.pose.position.x;
                    goal_y = transformed_msg.pose.position.y;
                    RCLCPP_INFO(this->get_logger(), "Transformed Goal from frame '%s' to '%s': (x=%.2f, y=%.2f)",
                        msg->header.frame_id.c_str(), map_frame_.c_str(), goal_x, goal_y);
                }
            } catch (const std::exception &ex) {
                RCLCPP_WARN(this->get_logger(), "TF goal transform note: using direct coords (x=%.2f, y=%.2f)", goal_x, goal_y);
            }
        }

        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "  NEW NAVIGATION GOAL: (%.2f, %.2f)", goal_x, goal_y);
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");

        // Cancel any existing navigation
        if (is_navigating_) {
            RCLCPP_INFO(this->get_logger(), "Cancelling previous navigation...");
            follow_path_client_->async_cancel_all_goals();
            is_navigating_ = false;
        }

        if (graph_.passages_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No passages available in graph!");
            return;
        }

        // 1. Get current robot pose
        double start_x = 0.0, start_y = 0.0;
        GetRobotPose(start_x, start_y);
        RCLCPP_INFO(this->get_logger(), "  Robot Start: (%.2f, %.2f)", start_x, start_y);

        // 2. Find closest passage (doorway) for start and goal
        PassageId start_id = FindClosestPassage(start_x, start_y);
        PassageId goal_id = FindClosestPassage(goal_x, goal_y);

        std::vector<Eigen::Vector3d> path_result;

        if (start_id == goal_id) {
            RCLCPP_INFO(this->get_logger(), "  Same room — direct path (Passage %ld)", start_id);
            path_result.push_back(Eigen::Vector3d(start_x, start_y, 0.0));
            path_result.push_back(Eigen::Vector3d(goal_x, goal_y, 0.0));
        } else {
            RCLCPP_INFO(this->get_logger(), "  Planning multi-room route: Door %ld → Door %ld...", start_id, goal_id);

            try {
                auto t_start = std::chrono::high_resolution_clock::now();
                PlanInPathGraph(graph_, start_id, goal_id, path_result);
                auto t_end = std::chrono::high_resolution_clock::now();
                double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

                if (path_result.empty()) {
                    RCLCPP_WARN(this->get_logger(), "  No valid route found between Door %ld and Door %ld!", start_id, goal_id);
                    // Fallback to connecting through the doorway centers
                    if (graph_.passages_.find(start_id) != graph_.passages_.end() && graph_.passages_.find(goal_id) != graph_.passages_.end()) {
                        path_result.push_back(graph_.passages_[start_id]->center_position);
                        path_result.push_back(graph_.passages_[goal_id]->center_position);
                    } else {
                        PublishStatus("FAILED: No route between rooms");
                        return;
                    }
                }

                // Prepend actual start and append actual goal
                path_result.insert(path_result.begin(), Eigen::Vector3d(start_x, start_y, 0.0));
                path_result.push_back(Eigen::Vector3d(goal_x, goal_y, 0.0));

                RCLCPP_INFO(this->get_logger(), "  Route planned in %.3f ms — %zu waypoints through doorways", elapsed_ms, path_result.size());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during PlanInPathGraph: %s", e.what());
                PublishStatus("ERROR: Exception during route planning");
                return;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception during PlanInPathGraph!");
                PublishStatus("ERROR: Route planning failure");
                return;
            }
        }

        // 3. Publish visualization markers
        PublishPath(path_result);

        // 4. Send path to Nav2 controller_server via FollowPath action
        SendPathToNav2(path_result, goal_x, goal_y);
    }

    void tf_stamped_transform(const geometry_msgs::msg::PoseStamped::SharedPtr in, geometry_msgs::msg::PoseStamped &out, const std::string &target_frame) {
        auto tf = tf_buffer_->lookupTransform(target_frame, in->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*in, out, tf);
    }

    // ========================================================================
    //  NAV2 INTEGRATION — Send planned path to controller_server for execution
    // ========================================================================
    void SendPathToNav2(const std::vector<Eigen::Vector3d>& waypoints, double goal_x, double goal_y) {
        // Check if the action server is available
        if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "══════════════════════════════════════════════════════════════");
            RCLCPP_ERROR(this->get_logger(), "  Nav2 controller_server NOT AVAILABLE!");
            RCLCPP_ERROR(this->get_logger(), "  Cannot navigate safely without obstacle avoidance.");
            RCLCPP_ERROR(this->get_logger(), "  Make sure the lifecycle_manager has activated all nodes.");
            RCLCPP_ERROR(this->get_logger(), "  Check: ros2 lifecycle get /controller_server");
            RCLCPP_ERROR(this->get_logger(), "══════════════════════════════════════════════════════════════");
            PublishStatus("ERROR: Nav2 controller not available");
            return;
        }

        // Build the nav_msgs/Path message for Nav2
        nav_msgs::msg::Path nav2_path;
        nav2_path.header.frame_id = map_frame_;
        nav2_path.header.stamp = this->now();

        for (size_t i = 0; i < waypoints.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = map_frame_;
            pose.header.stamp = this->now();
            pose.pose.position.x = waypoints[i][0];
            pose.pose.position.y = waypoints[i][1];
            pose.pose.position.z = 0.0;

            // Compute orientation from heading towards next waypoint
            if (i + 1 < waypoints.size()) {
                double dx = waypoints[i + 1][0] - waypoints[i][0];
                double dy = waypoints[i + 1][1] - waypoints[i][1];
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
            } else {
                // Final waypoint: face the same direction as the previous segment
                if (waypoints.size() >= 2) {
                    double dx = waypoints[i][0] - waypoints[i - 1][0];
                    double dy = waypoints[i][1] - waypoints[i - 1][1];
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, yaw);
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                } else {
                    pose.pose.orientation.w = 1.0;
                }
            }

            nav2_path.poses.push_back(pose);
        }

        // Create the FollowPath goal
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = nav2_path;
        goal_msg.controller_id = "FollowPath";       // Must match nav2_osmag_params.yaml
        goal_msg.goal_checker_id = "general_goal_checker";

        RCLCPP_INFO(this->get_logger(), "  Sending %zu-waypoint path to Nav2 controller_server...", nav2_path.poses.size());
        RCLCPP_INFO(this->get_logger(), "  Nav2 will use local_costmap + LiDAR for obstacle avoidance!");

        // Set up callbacks for goal response and result
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowPath::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Nav2 REJECTED the path! Check controller_server logs.");
                    is_navigating_ = false;
                    PublishStatus("REJECTED by Nav2");
                } else {
                    RCLCPP_INFO(this->get_logger(), "  Nav2 ACCEPTED the path! Robot is now moving with obstacle avoidance.");
                    is_navigating_ = true;
                    PublishStatus("NAVIGATING with obstacle avoidance");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleFollowPath::SharedPtr,
                   const std::shared_ptr<const FollowPath::Feedback> feedback) {
                // Log progress every ~5 seconds (Nav2 sends feedback at controller_frequency)
                static int feedback_count = 0;
                feedback_count++;
                if (feedback_count % 100 == 0) {  // Every 5 sec at 20Hz
                    double dist = feedback->distance_to_goal;
                    double speed = feedback->speed;
                    RCLCPP_INFO(this->get_logger(),
                        "  Nav2 Progress: dist_to_goal=%.2f m  speed=%.2f m/s", dist, speed);
                }
            };

        send_goal_options.result_callback =
            [this, goal_x, goal_y](const GoalHandleFollowPath::WrappedResult & result) {
                is_navigating_ = false;
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "★★★ GOAL REACHED at (%.2f, %.2f)! ★★★", goal_x, goal_y);
                        PublishStatus("GOAL REACHED!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(), "Navigation ABORTED by Nav2 (path blocked or robot stuck)");
                        PublishStatus("ABORTED - path blocked");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Navigation CANCELLED (new goal or user interrupt)");
                        PublishStatus("CANCELLED");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Navigation ended with unknown result code");
                        PublishStatus("UNKNOWN ERROR");
                        break;
                }
            };

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);
    }



    // ========================================================================
    //  STATUS MONITORING
    // ========================================================================
    void StatusCheck() {
        // Only log if navigating
        if (is_navigating_) {
            double rx = 0.0, ry = 0.0, ryaw = 0.0;
            if (GetRobotPoseWithYaw(rx, ry, ryaw)) {
                // Just heartbeat — detailed feedback comes from Nav2's callback
            }
        }
    }

    void PublishStatus(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    // ========================================================================
    //  TF HELPERS — get the robot pose from the TF tree
    // ========================================================================
    bool GetRobotPoseWithYaw(double &x, double &y, double &yaw) {
        try {
            std::string tf_err;
            geometry_msgs::msg::TransformStamped tf_stamped;

            if (tf_buffer_->canTransform(map_frame_, base_frame_, tf2::TimePointZero, &tf_err)) {
                tf_stamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            }
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
            marker.header.frame_id = map_frame_;
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
                marker.header.frame_id = map_frame_;
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
        path_msg.header.frame_id = map_frame_;
        path_msg.header.stamp = this->now();

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = map_frame_;
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
            pose.header.frame_id = map_frame_;
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
        RCLCPP_INFO(this->get_logger(), "  Published global path (%zu waypoints) to /osmag/global_path", path_msg.poses.size());
    }

    // ========================================================================
    //  MEMBER VARIABLES
    // ========================================================================
    std::string osm_file_path_;
    std::string map_frame_;
    std::string base_frame_;
    double resolution_;

    AreaGraph graph_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Nav2 action client
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Navigation state
    bool is_navigating_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OsmAgPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
