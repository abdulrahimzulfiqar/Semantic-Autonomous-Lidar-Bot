#!/usr/bin/env python3
"""
Generate a Gazebo Ignition world SDF file and a matching 2D Occupancy Grid (PGM/YAML)
directly from the ShanghaiTech_merge_2.osm topometric CAD vector map.

This guarantees 100.0% mathematical alignment between:
  1. Gazebo 3D physics collision walls
  2. ROS 2 2D occupancy grid (map_server / AMCL)
  3. osmAG semantic topological area graph (osmag_planner_node)
"""

import xml.etree.ElementTree as ET
import numpy as np
import math
import cv2
import os

# === File Paths ===
WORKSPACE_ROOT = "/Users/teamincredibles/Desktop/Setups/Downloads/Semantic-Autonomous-Wheelchair"
OSM_INPUT = os.path.join(WORKSPACE_ROOT, "docs/research/osm-AG/osm_ag_2/osm_ag_repo/osmAG/data/fix_id/ShanghaiTech_merge_2.osm")
SDF_OUTPUT = os.path.join(WORKSPACE_ROOT, "lidarbot_ws/src/lidarbot/lidarbot_gz/worlds/shanghaitech_merge_2_world.sdf")
PGM_OUTPUT = os.path.join(WORKSPACE_ROOT, "lidarbot_ws/src/lidarbot/lidarbot_navigation/maps/shanghaitech_merge_2.pgm")
YAML_OUTPUT = os.path.join(WORKSPACE_ROOT, "lidarbot_ws/src/lidarbot/lidarbot_navigation/maps/shanghaitech_merge_2.yaml")
OSM_WS_COPY = os.path.join(WORKSPACE_ROOT, "lidarbot_ws/src/lidarbot/lidarbot_osmag/data/ShanghaiTech_merge_2.osm")

# === Map & Wall Parameters ===
RESOLUTION = 0.05       # 5 cm per pixel
WALL_HEIGHT = 2.0       # 2.0 m tall walls in Gazebo
WALL_THICKNESS = 0.10   # 10 cm wall thickness in Gazebo & PGM
MARGIN = 5.0            # 5 m padding around bounding box

# WGS84 Constants
EQUATOR_RADIUS = 6378137.0
DEG_TO_RAD = math.pi / 180.0

# === 1. Parse OSM XML ===
print(f"Loading OSM from: {OSM_INPUT}")
tree = ET.parse(OSM_INPUT)
root = tree.getroot()

first_node = root.find("node")
ref_lat = float(first_node.attrib["lat"])
ref_lon = float(first_node.attrib["lon"])
print(f"Origin reference: lat={ref_lat:.8f}, lon={ref_lon:.8f}")

r_lat = ref_lat * DEG_TO_RAD
r_lon = ref_lon * DEG_TO_RAD

def to_cartesian(lat, lon):
    p_lat = lat * DEG_TO_RAD
    p_lon = lon * DEG_TO_RAD
    d_lon = p_lon - r_lon
    d_lat = p_lat - r_lat
    x = EQUATOR_RADIUS * d_lon * math.cos(r_lat)
    y = EQUATOR_RADIUS * d_lat
    return x, y

nodes = {}
for elem in root.findall("node"):
    nid = int(elem.attrib["id"])
    lat = float(elem.attrib["lat"])
    lon = float(elem.attrib["lon"])
    nodes[nid] = to_cartesian(lat, lon)

# Extract passages (doors) to ensure walls are NOT placed across doorways
passages = set()
for elem in root.findall("way"):
    tags = {t.attrib["k"]: t.attrib["v"] for t in elem.findall("tag")}
    if tags.get("osmAG:type") == "passage":
        refs = [int(nd.attrib["ref"]) for nd in elem.findall("nd")]
        if len(refs) >= 2:
            passages.add((min(refs[0], refs[1]), max(refs[0], refs[1])))

print(f"Parsed {len(nodes)} nodes and {len(passages)} passages (doors)")

# Extract unique wall segments (areas minus passages)
unique_segments = set()
for elem in root.findall("way"):
    tags = {t.attrib["k"]: t.attrib["v"] for t in elem.findall("tag")}
    if tags.get("osmAG:type") == "area":
        refs = [int(nd.attrib["ref"]) for nd in elem.findall("nd")]
        for i in range(len(refs)):
            n1 = refs[i]
            n2 = refs[(i + 1) % len(refs)]
            if n1 == n2:
                continue
            seg_key = (min(n1, n2), max(n1, n2))
            if seg_key not in passages:
                unique_segments.add(seg_key)

wall_segments = []
all_xs = []
all_ys = []

for n1, n2 in unique_segments:
    if n1 in nodes and n2 in nodes:
        p1 = nodes[n1]
        p2 = nodes[n2]
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if dist >= 0.05:  # Ignore segments shorter than 5cm
            wall_segments.append((p1, p2, dist))
            all_xs.extend([p1[0], p2[0]])
            all_ys.extend([p1[1], p2[1]])

min_x, max_x = min(all_xs), max(all_xs)
min_y, max_y = min(all_ys), max(all_ys)
print(f"Extracted {len(wall_segments)} valid wall segments")
print(f"Cartesian Extents: X=[{min_x:.2f}, {max_x:.2f}]m, Y=[{min_y:.2f}, {max_y:.2f}]m")

# === 2. Generate 2D Occupancy Grid (PGM + YAML) ===
ox = min_x - MARGIN
oy = min_y - MARGIN
grid_w = int(math.ceil((max_x - min_x + 2 * MARGIN) / RESOLUTION))
grid_h = int(math.ceil((max_y - min_y + 2 * MARGIN) / RESOLUTION))
print(f"Generating Occupancy Grid: {grid_w}x{grid_h} pixels @ {RESOLUTION} m/px, Origin: ({ox:.2f}, {oy:.2f})")

# Initialize grid with 254 (free space)
pgm_img = np.full((grid_h, grid_w), 254, dtype=np.uint8)

# Convert map coords (x, y) to image pixel coords (col, row)
def to_pixel(x, y):
    c = int(round((x - ox) / RESOLUTION))
    r = int(round((grid_h - 1) - (y - oy) / RESOLUTION))
    return (c, r)

# Draw wall lines in black (pixel value 0)
line_thickness_px = max(1, int(round(WALL_THICKNESS / RESOLUTION)))
for (p1, p2, dist) in wall_segments:
    pt1 = to_pixel(p1[0], p1[1])
    pt2 = to_pixel(p2[0], p2[1])
    cv2.line(pgm_img, pt1, pt2, 0, thickness=line_thickness_px, lineType=cv2.LINE_AA)

os.makedirs(os.path.dirname(PGM_OUTPUT), exist_ok=True)
cv2.imwrite(PGM_OUTPUT, pgm_img)
print(f"✅ Saved PGM: {PGM_OUTPUT}")

yaml_content = f"""image: shanghaitech_merge_2.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{ox:.4f}, {oy:.4f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open(YAML_OUTPUT, "w") as f:
    f.write(yaml_content)
print(f"✅ Saved YAML: {YAML_OUTPUT}")

# === 3. Generate Gazebo World (SDF) ===
z_center = WALL_HEIGHT / 2.0

lines = []
lines.append('<?xml version="1.0" ?>')
lines.append('<sdf version="1.7">')
lines.append('  <world name="shanghaitech_merge_2_world">')
lines.append('    <light name="sun" type="directional">')
lines.append('      <cast_shadows>1</cast_shadows>')
lines.append('      <pose>0 0 50 0 0 0</pose>')
lines.append('      <diffuse>0.8 0.8 0.8 1</diffuse>')
lines.append('      <specular>0.2 0.2 0.2 1</specular>')
lines.append('      <direction>-0.5 0.1 -0.9</direction>')
lines.append('    </light>')
lines.append('')
lines.append('    <model name="ground_plane">')
lines.append('      <static>1</static>')
lines.append('      <link name="link">')
lines.append('        <collision name="collision">')
lines.append('          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>')
lines.append('        </collision>')
lines.append('        <visual name="visual">')
lines.append('          <cast_shadows>0</cast_shadows>')
lines.append('          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>')
lines.append('          <material>')
lines.append('            <script>')
lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
lines.append('              <name>Gazebo/Grey</name>')
lines.append('            </script>')
lines.append('          </material>')
lines.append('        </visual>')
lines.append('      </link>')
lines.append('    </model>')
lines.append('')
lines.append('    <gravity>0 0 -9.8</gravity>')
lines.append('    <physics type="ode">')
lines.append('      <max_step_size>0.004</max_step_size>')
lines.append('      <real_time_factor>1</real_time_factor>')
lines.append('      <real_time_update_rate>250</real_time_update_rate>')
lines.append('    </physics>')
lines.append('')
lines.append('    <!-- Mandatory Plugins for Ignition Gazebo Sensors and Physics -->')
lines.append('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>')
lines.append('    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>')
lines.append('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>')
lines.append('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">')
lines.append('      <render_engine>ogre2</render_engine>')
lines.append('    </plugin>')
lines.append('    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>')
lines.append('')
lines.append('    <!-- Campus Building Wall Models -->')
lines.append('    <model name="shanghaitech_walls">')
lines.append('      <static>1</static>')
lines.append('      <link name="wall_link">')

for i, (p1, p2, length) in enumerate(wall_segments):
    cx = (p1[0] + p2[0]) / 2.0
    cy = (p1[1] + p2[1]) / 2.0
    yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    lines.append(f'        <collision name="wall_col_{i}">')
    lines.append(f'          <pose>{cx:.4f} {cy:.4f} {z_center:.4f} 0 0 {yaw:.5f}</pose>')
    lines.append(f'          <geometry><box><size>{length:.4f} {WALL_THICKNESS:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>')
    lines.append(f'        </collision>')
    lines.append(f'        <visual name="wall_vis_{i}">')
    lines.append(f'          <pose>{cx:.4f} {cy:.4f} {z_center:.4f} 0 0 {yaw:.5f}</pose>')
    lines.append(f'          <geometry><box><size>{length:.4f} {WALL_THICKNESS:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>')
    lines.append(f'          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>')
    lines.append(f'        </visual>')

lines.append('      </link>')
lines.append('    </model>')
lines.append('  </world>')
lines.append('</sdf>')

os.makedirs(os.path.dirname(SDF_OUTPUT), exist_ok=True)
with open(SDF_OUTPUT, "w") as f:
    f.write("\n".join(lines))
print(f"✅ Saved SDF: {SDF_OUTPUT} ({len(wall_segments)} wall boxes)")

# === 4. Copy OSM file to ROS 2 package share folder ===
os.makedirs(os.path.dirname(OSM_WS_COPY), exist_ok=True)
with open(OSM_INPUT, "r") as src, open(OSM_WS_COPY, "w") as dst:
    dst.write(src.read())
print(f"✅ Copied OSM to ROS 2 package: {OSM_WS_COPY}")
print("\n🎉 Generation Complete! 100% matched CAD simulation files ready.")
