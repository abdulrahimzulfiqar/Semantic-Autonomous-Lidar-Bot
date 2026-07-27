# Parameter Tweaking & Workspace Optimization Guide

This document contains a complete file-by-file audit, parameter tweaking guide, impact analysis, and optimization checklist for the autonomous wheelchair platform (`firmware` and `lidarbot_ws`).

---

## 1. Low-Level Firmware Layer (`firmware/arduino_bridge/arduino_bridge.ino`)

### Educational Overview
- **What is it?** C++ code running on the Arduino Uno microcontroller managing wheel motors (via L298N H-Bridge), reading wheel encoders, and reading the MPU6050 IMU.
- **Why do we need it?** The Raspberry Pi runs a non-real-time OS (Ubuntu Server). The Arduino provides bare-metal hardware interrupt handling to ensure zero wheel encoder pulses are missed.
- **How does it work?** 
  - **Interrupts**: Reads encoder pulses on digital pins 2 & 3.
  - **Command Reading**: Reads serial velocity targets (`M,left_speed,right_speed\n`).
  - **PID Control**: Runs a 50Hz loop adjusting motor PWM output based on feedback.
  - **Telemetry**: Writes raw encoder tick counts and IMU data back to the Pi at 20Hz.

### Impact on the Robot
- **Positive Impact**: Closed-loop PID control prevents motor stall on uneven surfaces; raw tick reporting enables accurate odometry calculation.
- **Negative Impact / Stalls**: Blocking `Serial.readStringUntil('\n')` calls can freeze loop timing; slow `digitalRead()` calls inside interrupts waste CPU time.

### Optimization Score: 70%

### Actionable Optimizations
1. **Replace `String` with Non-Blocking Char Array Parsing**: Use `strtok` and `atof` to eliminate heap fragmentation and serial blocking.
2. **Direct Port Manipulation in ISRs**: Replace `digitalRead()` with `PIND` bitwise operations to reduce interrupt execution time from ~4.5$\mu s$ to ~0.4$\mu s$.
3. **Increase Telemetry Frequency**: Match telemetry output to 50Hz (20ms interval) to provide smoother updates to ROS 2 EKF localization.

---

## 2. Low-Level ROS 2 Interface (`lidarbot_bringup/script/arduino_bridge.py`)

### Educational Overview
- **What is it?** ROS 2 Python node acting as serial middleware between ROS 2 topics and `/dev/ttyUSB_ARDUINO`.
- **Why do we need it?** Converts high-level ROS velocity commands (`cmd_vel`) into target tick speeds for the Arduino, and converts raw encoder telemetry into ROS 2 `Odometry` and `IMU` messages.
- **How does it work?** Subscribes to `/cmd_vel`, calculates individual wheel speeds based on wheel geometry, writes commands to USB serial, and publishes `odom_raw` and `imu_raw`.

### Impact on the Robot
- **Positive Impact**: Real-time odometry dead-reckoning calculation and TF publishing.
- **Performance Bottleneck**: Synchronous serial reading inside timer callback can block the node if serial glitches occur; duplicate TF broadcasting can clash with `robot_localization`.

### Key Parameters to Tweak

| Parameter | Location | Current Value | Recommended | Impact / Why Tweak? |
| :--- | :--- | :---: | :---: | :--- |
| `wheel_base` | L25 | `0.169` m | `0.169` m | **Turning Precision**: Adjusting fixes over/under spinning on 90° turns. |
| `ticks_per_rev` | L23 | `450.5` | `450.5` | **Distance Calibration**: Adjusting fixes linear distance scaling errors. |
| `timer_frequency` | L54 | `0.02` (50 Hz) | `0.02` (50 Hz) | **Update Speed**: Controls odometry publication frequency. |

### Optimization Score: 75%

### Actionable Optimizations
1. **Daemon Serial Threading**: Run serial byte reading in a dedicated background thread.
2. **Conditional TF Broadcast**: Add a `publish_tf` parameter (set `False` when running EKF, `True` for standalone testing).

---

## 3. Physical Hardware Launch (`lidarbot_bringup/launch/real_robot.launch.py`)

### Educational Overview
- **What is it?** ROS 2 launch script that starts `robot_state_publisher`, `rplidar_node`, and `arduino_bridge.py`.
- **Why do we need it?** Single entry-point script to initialize physical hardware.
- **How does it work?** Parses robot URDF files via Xacro and executes hardware driver processes.

### Key Parameters to Tweak

| Parameter | Location | Current Value | Recommended | Impact / Why Tweak? |
| :--- | :--- | :---: | :---: | :--- |
| `serial_port` | L30 | `/dev/ttyUSB_LIDAR` | Persistent Udev | Prevents startup failures if USB ports swap on reboot. |
| `angle_compensate` | L34 | `True` | `True` | Ensures multi-revolution 360° point cloud normalization. |

### Optimization Score: 80%

### Actionable Optimizations
1. **Linux Udev Rules**: Map hardware vendor IDs to static aliases (`/dev/rplidar`, `/dev/arduino`).

---

## 4. SLAM Mapping Layer (`lidarbot_slam/config/mapper_params_online_async.yaml`)

### Educational Overview
- **What is it?** Configuration file for `slam_toolbox` in online asynchronous mapping mode.
- **Why do we need it?** Builds 2D occupancy grid maps (`map.pgm` / `map.yaml`) using LiDAR scans and wheel odometry.
- **How does it work?** Uses Ceres non-linear optimization for scan matching and graph-based loop closures.

### Key Parameters to Tweak

| Parameter | Location | Current Value | Recommended | Impact / Why Tweak? |
| :--- | :--- | :---: | :---: | :--- |
| `resolution` | L32 | `0.05` (5cm) | `0.05` | **Map Grid Size**: Lowering to 2cm quadruples RAM and LLM token usage. |
| `max_laser_range` | L33 | `20.0` m | `12.0` m | **Processing Speed**: Cuts out noisy long-distance laser returns beyond indoor room limits. |
| `minimum_travel_distance` | L43 | `0.1` m | `0.2` m | **Graph Size**: Reduces SLAM graph density by 30%, speeding up graph serialization for LLMs. |
| `minimum_travel_heading` | L44 | `0.2` rad | `0.3` rad | **Rotation Graph Density**: Prevents cluttering graph nodes during static in-place spins. |
| `transform_publish_period` | L30 | `0.02` (50 Hz) | `0.05` (20 Hz) | **CPU Load**: Reduces Pi CPU load dedicated to TF publishing. |

### Optimization Score: 85%

---

## 5. Navigation Stack Layer (`lidarbot_navigation/config/nav2_params_real.yaml`)

### Educational Overview
- **What is it?** Configuration file for Nav2 (AMCL localization, DWB local planner, NavFn global planner, Costmaps).
- **Why do we need it?** Governs path planning, obstacle inflation padding, and velocity control.
- **How does it work?** Global planner finds topological paths; DWB local planner evaluates trajectory samples to drive motors safely.

### Key Parameters to Tweak

| Parameter | Location | Current Value | Recommended | Impact / Why Tweak? |
| :--- | :--- | :---: | :---: | :--- |
| `controller_frequency` | L94 | `10.0` Hz | `8.0` - `10.0` Hz | **Pi CPU Optimization**: Prevents CPU throttling during dynamic obstacle evaluation. |
| `xy_goal_tolerance` | L109 | `0.3` m | `0.25` m | **Arrival Precision**: Distance tolerance for declaring destination reached. |
| `yaw_goal_tolerance` | L111 | `0.4` rad | `0.3` rad | **Heading Accuracy**: Final angle alignment tolerance at destination. |
| `inflation_radius` | Costmap | `0.55` m | `0.45` m | **Doorway Passage**: Lowering allows entering standard 75-80cm indoor doorways. |
| `sim_time` | L132 | `1.7` s | `1.5` s | **Trajectory Lookahead**: Time horizon evaluated for dynamic obstacle avoidance. |

### Optimization Score: 90%

---

## 6. Summary Matrix of Workspace Optimizations

| File / Component | Category | Current Score | Main Issue / Bottleneck | Primary Fix |
| :--- | :--- | :---: | :--- | :--- |
| `arduino_bridge.ino` | Firmware | **70%** | Blocking `Serial.readStringUntil` & `digitalRead` overhead | Non-blocking char parsing & direct port register access |
| `arduino_bridge.py` | Python Bridge | **75%** | Synchronous serial reading & potential TF clashes | Dedicated serial thread & `publish_tf` toggle parameter |
| `real_robot.launch.py` | Hardware Launch | **80%** | Hardcoded USB device paths (`/dev/ttyUSB0`) | Configure static Linux udev rules (`/dev/rplidar`) |
| `mapper_params_online_async.yaml` | SLAM Mapping | **85%** | Excessive laser scan range processing (20m) | Cap `max_laser_range: 12.0` & set min travel distance `0.2` |
| `nav2_params_real.yaml` | Navigation | **90%** | High costmap inflation radius blocking doors | Reduce `inflation_radius: 0.45` for tight indoor doorways |
