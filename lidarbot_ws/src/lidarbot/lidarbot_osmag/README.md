# lidarbot_osmag

ROS 2 Humble package for semantic hierarchical indoor global path planning using `osmAG` (Area Graph & Dual Passage Graph).

## Overview

The `lidarbot_osmag` package wraps the high-performance C++ `osmAG` engine into a standalone ROS 2 Humble node. It parses OpenStreetMap XML files (`.osm`) containing 3D area polygons and passage doorways, pre-computes intra-area door-to-door 2D A* walking distances, and executes multi-building global path planning in **under 3 milliseconds**.

## ROS 2 Interfaces

### Subscribed Topics
- `/goal_pose` (`geometry_msgs/msg/PoseStamped`): Navigation goal pose set via Foxglove Studio or RViz2.

### Published Topics
- `/osmag/map_markers` (`visualization_msgs/msg/MarkerArray`): 3D area polygon boundaries (green lines) and passage doorways (red lines) for visualization.
- `/osmag/global_path` (`nav_msgs/msg/Path`): 3D metric trajectory (sequence of waypoints) for the Nav2 local controller.
- `/osmag/path_marker` (`visualization_msgs/msg/Marker`): Thick magenta route line for 3D visualization.

## Configuration Parameters

| Parameter | Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `osm_file_path` | string | `""` | Path to the input `.osm` AreaGraph file |
| `map_frame` | string | `"map"` | TF frame ID for published ROS 2 messages |
| `resolution` | double | `0.1` | Meters per pixel resolution for 2D grid submaps |

## How to Run Standalone

1. **Build the Package**:
   ```bash
   cd lidarbot_ws
   colcon build --packages-select lidarbot_osmag --symlink-install
   source install/setup.zsh
   ```

2. **Launch the Node**:
   ```bash
   ros2 launch lidarbot_osmag osmag_launch.py
   ```

3. **Send a Goal Pose**:
   In RViz2 or Foxglove Studio, click the **"2D Goal Pose"** tool to send a target position on `/goal_pose`.


## GCU map readiness (before autonomous navigation)

The reconstructed CAD export contains area relations and multi-node interfaces
that the older loader cannot consume directly. Generate the simple-way GCU
adapter with `scripts/prepare_gcu_simulation.py` from the project root. The parser
now resolves named area/parent references and rejects unsupported area relations.

After building on ROS 2 Linux, run:

```bash
ros2 run lidarbot_osmag osmag_map_check /absolute/path/to/gcu_ground_floor.osm --expect-root-excluded
```

This checks the real parser and topology without sending commands. Gazebo,
footprint feasibility and route execution are separate gates; see
`docs/research/osm-AG/GCU_SIMULATION_READINESS.md`. Do not treat a connected
point-based graph as proof of wheelchair clearance.
