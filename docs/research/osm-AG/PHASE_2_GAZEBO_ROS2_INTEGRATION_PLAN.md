# Phase 2 Implementation Plan: ROS 2 Gazebo Simulation & Nav2 osmAG Integration

This document outlines the complete architectural design, file structure, integration steps, testing strategy, and expected outcomes for **Phase 2: Practical ROS 2 Gazebo Simulation & Nav2 osmAG Bringup**.

---

## 1. Overview & Objectives

### **What is Phase 2?**
Phase 2 transitions our working C++ `osmAG` topometric planning engine from standalone offline tests into an **active, real-time ROS 2 Humble robotic system** operating inside a Gazebo physics simulation (and subsequently on the physical wheelchair robot).

### **Why do we need it?**
Before adding higher-level cognitive layers like LLM Copilots (Step 4) or Open-Vocabulary Object Search (Step 5), the physical robot must be able to:
1. Localize itself in 2D space (`/tf`, `map` $\rightarrow$ `odom` $\rightarrow$ `base_link`).
2. Receive global topometric routes from `lidarbot_osmag` as standard ROS 2 paths (`nav_msgs/msg/Path`).
3. Execute real-time obstacle avoidance and path tracking using **Nav2 local controllers** (DWB / Regulated Pure Pursuit) to drive the simulated/physical wheels via `/cmd_vel`.

### **How does it work?**
```
                       ┌───────────────────────────────┐
                       │   RViz2 / Foxglove Studio     │
                       │   User selects 2D Goal Pose   │
                       └───────────────┬───────────────┘
                                       │ /goal_pose (geometry_msgs/PoseStamped)
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           lidarbot_osmag Node                                │
│                                                                              │
│ 1. Looks up current robot pose via TF (/tf: map -> base_link).               │
│ 2. Identifies current Start Room ID and Target Goal Room ID in osmAG XML.    │
│ 3. Computes multi-building topometric shortest path via Dual Passage Graph.  │
│ 4. Pre-computes and joins 2D A* sub-trajectories between doorways.           │
│ 5. Publishes 3D Room Polygons & Doorways on /osmag/map_markers.              │
│ 6. Publishes metric path waypoints on /osmag/global_path.                    │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │ /osmag/global_path (nav_msgs/Path)
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                    Nav2 Controller Server (Local Planner)                    │
│                                                                              │
│ • Controller: RegulatedPurePursuit / DWB Local Planner                       │
│ • Local Costmap: Live 2D LiDAR Scan (/scan) for dynamic obstacle avoidance   │
│ • Output: Velocity commands (/cmd_vel)                                       │
└──────────────────────────────────────┬───────────────────────────────────────┘
                                       │ /cmd_vel (geometry_msgs/Twist)
                                       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                   Gazebo Simulation / Real Robot Hardware                    │
│                                                                              │
│ • Diff Drive / Wheelchair Motor Controller                                   │
│ • 2D LiDAR Sensor (Ray Plugin -> /scan)                                      │
│ • Wheel Odometry (/odom -> /tf)                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. File & Directory Structure

Here is the exact file manifest showing the packages we will create, modify, and configure in `lidarbot_ws/src/lidarbot/`:

```
lidarbot_ws/src/lidarbot/
├── lidarbot_osmag/                      # [CORE] osmAG ROS 2 Integration Node
│   ├── CMakeLists.txt                   # Compiles osmag_planner_node with C++17 & OpenCV
│   ├── package.xml                      # ROS 2 Humble dependencies (rclcpp, nav_msgs, tf2)
│   ├── README.md                        # Package documentation & test instructions
│   ├── config/
│   │   └── osmag_params.yaml            # Configurable parameters (map path, frame IDs, resolution)
│   ├── data/
│   │   ├── big_map_7.osm                # Our custom multi-room osmAG XML map
│   │   └── ShanghaiTech_merge_2.osm     # Multi-building benchmark osmAG XML map
│   ├── include/osmAG/                   # Self-contained osmAG C++ engine headers
│   │   ├── areagraph.h
│   │   ├── astarcal.h
│   │   ├── data_load_save.h
│   │   ├── passage.h
│   │   ├── pathgraph.h
│   │   └── visualization.h
│   ├── launch/
│   │   └── osmag_launch.py              # Standalone osmAG launch file
│   └── src/
│       ├── osmAG/                       # Self-contained osmAG C++ implementation
│       │   ├── astarcal.cpp
│       │   ├── data_load_save.cpp
│       │   └── path_graph.cpp
│       └── osmag_planner_node.cpp       # ROS 2 Node (TF listener, /goal_pose sub, /osmag/global_path pub)
│
├── lidarbot_navigation/                 # [NAV2] Nav2 Stack & Local Controller Integration
│   ├── config/
│   │   ├── nav2_osmag_params.yaml       # Nav2 controller server tuned for osmAG paths
│   │   └── amcl_params.yaml             # 2D LiDAR localization parameters
│   ├── launch/
│   │   ├── navigation_osmag_launch.py   # Launches Nav2 Controller + AMCL + osmAG Node together
│   │   └── rviz_osmag_launch.py         # Launches RViz2 with pre-configured osmAG displays
│   ├── rviz/
│   │   └── osmag_navigation.rviz        # RViz2 display config (Map, MarkerArray, Path, RobotModel)
│   └── maps/
│       ├── big_map_7.yaml               # 2D Grid map metadata
│       └── big_map_7_cleaned.png        # 2D Grid map occupancy image
│
├── lidarbot_gazebo/                     # [SIMULATION] Gazebo Worlds & Physics Setup
│   ├── launch/
│   │   └── gazebo_big_map_7_launch.py   # Spawns robot into Gazebo big_map_7 simulated world
│   └── worlds/
│       └── big_map_7_world.world        # Gazebo 3D simulation world matching big_map_7 layout
│
└── lidarbot_bringup/                    # [FULL BRINGUP] Complete One-Click Launch Suite
    └── launch/
        └── sim_osmag_bringup_launch.py  # Launches Gazebo + Robot State + Nav2 + osmAG + RViz
```

---

## 3. Step-by-Step Implementation Sequence

### **Step 2.1: Enhance `lidarbot_osmag` Node for Live TF Tracking**
- Modify `osmag_planner_node.cpp` to use a `tf2_ros::Buffer` and `TransformListener`.
- When a `/goal_pose` is received, look up the transform `map` $\rightarrow$ `base_link` to get the robot's real-time Cartesian start position $(x_s, y_s)$ automatically, rather than relying on a hardcoded start node.
- Transform the goal pose $(x_g, y_g)$ into the `osmAG` coordinate frame.
- Execute dual passage routing + 2D A* waypoint extraction and publish `nav_msgs/msg/Path` with accurate timestamps and `frame_id = "map"`.

### **Step 2.2: Gazebo World Construction (`big_map_7_world.world`)**
- Create a Gazebo simulation world matching the geometry of `big_map_7_cleaned.png` (walls, 4 rooms, doorways, corridor).
- Configure lighting, ground plane, and collision physics parameters.
- Verify simulated 2D LiDAR ray casting accurately detects the simulated walls and obstacles.

### **Step 2.3: Nav2 Controller Server Tuning (`nav2_osmag_params.yaml`)**
- Configure the **Nav2 Controller Server** (`nav2_controller::ControllerServer`) to accept `/osmag/global_path` as its reference path.
- Tune the **Regulated Pure Pursuit Controller** / **DWB Controller**:
  - Max linear velocity: $0.4\text{ m/s}$ (safe indoor wheelchair speed).
  - Max angular velocity: $0.8\text{ rad/s}$.
  - Lookahead distance: $0.4\text{ m} \sim 0.8\text{ m}$.
  - Local Costmap: Live 2D LiDAR scan integration with inflation radius of $0.25\text{ m}$ for dynamic obstacle avoidance.

### **Step 2.4: Unified Bringup Launch (`sim_osmag_bringup_launch.py`)**
- Create a master launch file that coordinates:
  1. Gazebo simulation with `big_map_7_world.world`.
  2. Robot State Publisher (`robot_description` URDF with LiDAR & differential drive plugins).
  3. Map Server + AMCL (2D Localization).
  4. `lidarbot_osmag` Planner Node.
  5. Nav2 Controller Server (Local execution & collision prevention).
  6. RViz2 / Foxglove visualization.

---

## 4. How We Will Test It (Verification Protocol)

We will verify Phase 2 systematically across 4 testing stages:

### **Test Stage A: Standalone `lidarbot_osmag` Node Test**
```bash
# Terminal 1: Launch osmAG node with big_map_7.osm
ros2 launch lidarbot_osmag osmag_launch.py osm_file_path:=/path/to/big_map_7.osm

# Terminal 2: Publish a mock goal pose
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 4.5, y: 1.2, z: 0.0}, orientation: {w: 1.0}}
}"

# Terminal 3: Echo published global path
ros2 topic echo /osmag/global_path --once
```
- **Success Criteria**: Node logs successful passage route discovery, prints execution time ($< 3\text{ ms}$), and publishes a populated `nav_msgs/msg/Path` message.

---

### **Test Stage B: Gazebo Simulation World & Sensor Verification**
```bash
# Launch Gazebo world with robot model
ros2 launch lidarbot_gazebo gazebo_big_map_7_launch.py
```
- **Success Criteria**: Gazebo launches without physics errors; robot spawns at origin; `/scan` publishes live laser data; `/odom` publishes wheel odometry.

---

### **Test Stage C: RViz / Foxglove Visual Display Verification**
- **Displays to Enable**:
  - `Map` $\rightarrow$ Topic: `/map`
  - `MarkerArray` $\rightarrow$ Topic: `/osmag/map_markers` (Green 3D room polygons and Red doorways).
  - `Path` $\rightarrow$ Topic: `/osmag/global_path` (Magenta trajectory).
  - `LaserScan` $\rightarrow$ Topic: `/scan`
- **Success Criteria**: The room graph boundary lines and doorway markers render perfectly aligned over the 2D grid map canvas.

---

### **Test Stage D: Full End-to-End Navigation Test in Gazebo**
```bash
# Launch complete integrated simulation bringup
ros2 launch lidarbot_bringup sim_osmag_bringup_launch.py
```
1. In RViz2, use the **"2D Goal Pose"** tool to click inside Room 4.
2. Observe `lidarbot_osmag` compute the multi-room path through doorways.
3. Observe the Nav2 controller steer the simulated robot wheels along the trajectory.
4. Spawn a dynamic obstacle in Gazebo in the robot's direct path.
5. **Success Criteria**: The robot drives smoothly through the doorways to the destination room, slowing down or navigating around the dynamic obstacle using local costmap LiDAR rays.

---

## 5. What to Expect (Expected Outputs & Metrics)

| Component / Output | Topic / Metric | Expected Value / Visual |
|---|---|---|
| **Topometric Markers** | `/osmag/map_markers` | 3D green lines for 4 rooms/8 cells; 3 red line segments for doorways. |
| **Global Path** | `/osmag/global_path` | Smooth metric waypoint sequence connecting start room to goal room. |
| **Planning Latency** | Node Console Output | **$< 3.0\text{ ms}$** total computation time. |
| **Motion Commands** | `/cmd_vel` | Smooth linear ($0.0 \sim 0.4\text{ m/s}$) and angular velocities. |
| **Navigation Accuracy** | Goal Tolerance | Final stopping position within **$< 0.15\text{ m}$** of target pose. |

---

## 6. Next Steps After Phase 2 Completion

Once the Gazebo simulation and Nav2 bringup are verified:
- **Phase 3 (Step 3 & 4)**: Implement the **LLM Copilot Node** (`lidarbot_llm`) to accept natural language voice/text commands ("Go to Office 2") and parse campus maintenance notices to dynamically block doors in `osmAG`.
- **Phase 4 (Step 5)**: Implement **Zero-Shot Object Navigation** with camera vision (YOLO/CLIP).
