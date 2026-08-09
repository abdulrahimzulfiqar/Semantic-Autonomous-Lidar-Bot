/**
 * @file osmag_planner_node.cpp
 * @brief ROS 2 Humble C++ Node for osmAG Semantic Hierarchical Global Path Planning
 * 
 * Subscribes:
 *   - /goal_pose (geometry_msgs/msg/PoseStamped): Target navigation goal from Foxglove Studio / RViz2
 * 
 * Publishes:
 *   - /osmag/map_markers (visualization_msgs/msg/MarkerArray): 3D area polygons & doorway lines
 *   - /osmag/global_path (nav_msgs/msg/Path): Hierarchical planned trajectory for Nav2 controller
 *   - /osmag/path_marker (visualization_msgs/msg/Marker): 3D thick route line for Foxglove Studio
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "data_load_save.h"
#include "pathgraph.h"
#include "area_grid_map.h"
#include "visualization.h"

using namespace osm_ag;

class OsmAgPlannerNode : public rclcpp::Node {
public:
    OsmAgPlannerNode() : Node("osmag_planner_node") {
        // Declare parameters
        this->declare_parameter<std::string>("osm_file_path", "");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<double>("resolution", 0.1);

        this->get_parameter("osm_file_path", osm_file_path_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("resolution", resolution_);

        if (osm_file_path_.empty() || osm_file_path_[0] != '/') {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("lidarbot_osmag");
            if (osm_file_path_.empty()) {
                osm_file_path_ = pkg_share + "/data/ShanghaiTech_merge_2.osm";
            } else {
                osm_file_path_ = pkg_share + "/" + osm_file_path_;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initializing osmAG Planner Node...");
        RCLCPP_INFO(this->get_logger(), "Loading OSM File: %s", osm_file_path_.c_str());

        // Parse OSM-AG File
        if (!Load_Osm2AreaGraph(osm_file_path_, graph_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", osm_file_path_.c_str());
            return;
        }

        TraverseArea(graph_);
        TraversePassage(graph_);

        RCLCPP_INFO(this->get_logger(), "Parsed %zu areas and %zu passages successfully!",
                    graph_.areas_.size(), graph_.passages_.size());

        // Pre-compute 2D grid submaps for all leaf areas
        RCLCPP_INFO(this->get_logger(), "Building 2D Grid Submaps for leaf areas...");
        for (auto& pair : graph_.areas_) {
            InitOccupancyMap_Area(graph_, pair.first, resolution_);
        }
        RCLCPP_INFO(this->get_logger(), "2D Grid Submaps initialization complete!");

        // Publishers with Transient Local QoS for visualization persistence
        auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
        
        map_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/osmag/map_markers", qos_transient);
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/osmag/global_path", rclcpp::QoS(10).reliable());
        path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/osmag/path_marker", rclcpp::QoS(10).reliable());

        // Subscriber for target goal pose
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&OsmAgPlannerNode::GoalCallback, this, std::placeholders::_1));

        // Publish campus map 3D markers for Foxglove Studio / RViz2
        PublishMapMarkers();

        RCLCPP_INFO(this->get_logger(), "osmAG Planner Node ready! Send a goal pose on /goal_pose to plan a route.");
    }

private:
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Received Nav Goal: (%.2f, %.2f)", goal_x, goal_y);

        if (graph_.passages_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No passages available in graph!");
            return;
        }

        // Find passage closest to goal position
        PassageId start_id = graph_.passages_.begin()->first;
        PassageId goal_id = FindClosestPassage(goal_x, goal_y);

        // Test planning from default start passage to goal passage
        auto it = graph_.passages_.begin();
        std::advance(it, std::min<size_t>(45, graph_.passages_.size() - 1));
        start_id = it->first;

        RCLCPP_INFO(this->get_logger(), "Planning route from Passage %ld to Passage %ld...", start_id, goal_id);

        std::vector<Eigen::Vector3d> path_result;
        auto t_start = std::chrono::high_resolution_clock::now();
        PlanInPathGraph(graph_, start_id, goal_id, path_result);
        auto t_end = std::chrono::high_resolution_clock::now();

        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        RCLCPP_INFO(this->get_logger(), "Plan completed in %.3f ms! Generated %zu 3D waypoints.", elapsed_ms, path_result.size());

        if (!path_result.empty()) {
            PublishPath(path_result);
        }
    }

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
            marker.scale.x = 0.15; // line width
            marker.color.r = 0.0f;
            marker.color.g = 0.8f;
            marker.color.b = 0.2f;
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
                marker.points.push_back(marker.points.front()); // Close polygon loop
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
                marker.scale.x = 0.3; // doorway thickness
                marker.color.r = 1.0f;
                marker.color.g = 0.1f;
                marker.color.b = 0.1f;
                marker.color.a = 1.0f;

                geometry_msgs::msg::Point p1, p2;
                p1.x = graph_.nodes_[src_id]->attributes_->position[0];
                p1.y = graph_.nodes_[src_id]->attributes_->position[1];
                p1.z = 0.1;
                p2.x = graph_.nodes_[tgt_id]->attributes_->position[0];
                p2.y = graph_.nodes_[tgt_id]->attributes_->position[1];
                p2.z = 0.1;

                marker.points.push_back(p1);
                marker.points.push_back(p2);
                marker_array.markers.push_back(marker);
            }
        }

        map_markers_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu map markers to /osmag/map_markers", marker_array.markers.size());
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
        line_marker.scale.x = 0.5; // thick route line
        line_marker.color.r = 1.0f; // Magenta color
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
            marker_pt.z = pt[2] + 0.2; // slightly elevated above ground
            line_marker.points.push_back(marker_pt);
        }

        global_path_pub_->publish(path_msg);
        path_marker_pub_->publish(line_marker);

        RCLCPP_INFO(this->get_logger(), "Published global path with %zu poses to /osmag/global_path", path_msg.poses.size());
    }

    std::string osm_file_path_;
    std::string map_frame_;
    double resolution_;

    AreaGraph graph_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OsmAgPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
