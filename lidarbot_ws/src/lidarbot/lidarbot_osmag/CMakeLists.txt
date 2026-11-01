cmake_minimum_required(VERSION 3.10)
project(lidarbot_osmag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_definitions(-DOSMAG_LIBRARY_BUILD)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Find tinyxml2 (no CMake config on Ubuntu 22.04, use manual find)
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
find_library(TINYXML2_LIBRARY tinyxml2)
if(NOT TINYXML2_INCLUDE_DIR OR NOT TINYXML2_LIBRARY)
  message(FATAL_ERROR "tinyxml2 not found. Install with: sudo apt install libtinyxml2-dev")
endif()

include_directories(
  include
  include/osmAG
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TINYXML2_INCLUDE_DIR}
)

# Build self-contained osmag core static library
add_library(osmag_lib STATIC
  src/osmAG/path_graph.cpp
  src/osmAG/astarcal.cpp
  src/osmAG/area.cpp
  src/osmAG/node.cpp
  src/osmAG/data_load_save.cpp
)
target_include_directories(osmag_lib PUBLIC include/osmAG ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIR})
target_link_libraries(osmag_lib ${OpenCV_LIBS} ${TINYXML2_LIBRARY})

# Build ROS 2 Node
add_executable(osmag_planner_node src/osmag_planner_node.cpp)
target_link_libraries(osmag_planner_node
  osmag_lib
  ${OpenCV_LIBS}
  ${TINYXML2_LIBRARY}
)
ament_target_dependencies(osmag_planner_node
  rclcpp
  rclcpp_action
  ament_index_cpp
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Offline GCU contract checker uses the real C++ loader without ROS initialization.
add_executable(osmag_map_check src/osmag_map_check.cpp)
target_link_libraries(osmag_map_check osmag_lib)

# Install Targets
install(TARGETS
  osmag_lib
  osmag_planner_node
  osmag_map_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
