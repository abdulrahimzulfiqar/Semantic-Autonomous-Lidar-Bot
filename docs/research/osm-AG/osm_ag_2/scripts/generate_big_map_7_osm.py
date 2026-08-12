#!/usr/bin/env python3
import xml.etree.ElementTree as ET
from xml.dom import minidom
import cv2
import numpy as np
import os

def create_osmag_xml(output_path):
    # Reference geodetic origin for big_map_7
    ref_lat = 31.177153641
    ref_lon = 121.594053967
    
    # big_map_7 properties (from big_map_7.yaml)
    res = 0.05
    orig_x = -9.46
    orig_y = -3.37
    
    # Helper to convert pixel (x, y) to WGS84 lat, lon
    def px_to_latlon(px_x, px_y):
        x_m = (px_x * res) + orig_x
        y_m = ((151 - px_y) * res) + orig_y  # Flip y for standard cartesian
        lat = ref_lat + (y_m / 111139.0)
        lon = ref_lon + (x_m / (111139.0 * np.cos(np.radians(ref_lat))))
        return lat, lon

    root = ET.Element("osm", version="0.6", generator="osmAG_Generator")
    
    # 1. Root Origin Node (with root="true" attribute required by osmAG parser)
    origin_node = ET.SubElement(root, "node", id="-1000", action="modify", visible="true",
                                 lat=f"{ref_lat:.9f}", lon=f"{ref_lon:.9f}",
                                 root="true", alt="0", qx="0", qy="0", qz="0")
    ET.SubElement(origin_node, "tag", k="osmAG:type", v="origin")
    
    node_id_counter = -1001
    way_id_counter = -2001
    
    # Extract room areas from big_map_7_cleaned image
    img = cv2.imread('docs/research/osm-AG/maps/big_map_7_cleaned.png', cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    
    # Define room boxes corresponding to the segmented layout of big_map_7
    room_boxes = [
        ("Room_1", 10, 10, w//2 - 5, h//2 - 10),
        ("Room_2", w//2 + 5, 10, w - 10, h//2 - 10),
        ("Room_3", 10, h//2 + 10, w//2 - 5, h - 10),
        ("Room_4", w//2 + 5, h//2 + 10, w - 10, h - 10),
        ("Corridor_Main", w//4, h//2 - 15, 3*w//4, h//2 + 15)
    ]
    
    area_nodes_map = {}
    area_ids = []
    
    for name, x1, y1, x2, y2 in room_boxes:
        way_id = str(way_id_counter)
        way_id_counter -= 1
        area_ids.append((name, way_id))
        
        pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        way_node_ids = []
        
        for px_x, px_y in pts:
            lat, lon = px_to_latlon(px_x, px_y)
            nid = str(node_id_counter)
            node_id_counter -= 1
            
            ET.SubElement(root, "node", id=nid, action="modify", visible="true",
                           lat=f"{lat:.9f}", lon=f"{lon:.9f}")
            way_node_ids.append(nid)
            
        way_elem = ET.SubElement(root, "way", id=way_id, action="modify", visible="true")
        for nid in way_node_ids:
            ET.SubElement(way_elem, "nd", ref=nid)
        # Close loop
        ET.SubElement(way_elem, "nd", ref=way_node_ids[0])
        
        ET.SubElement(way_elem, "tag", k="osmAG:type", v="area")
        ET.SubElement(way_elem, "tag", k="osmAG:areaType", v="room" if "Room" in name else "corridor")
        ET.SubElement(way_elem, "tag", k="osmAG:level", v="1")
        ET.SubElement(way_elem, "tag", k="name", v=name)
        
        area_nodes_map[way_id] = (pts, way_node_ids)

    # 2. Passages connecting rooms
    passages = [
        (area_ids[0][1], area_ids[4][1], (w//3, h//2 - 10), (w//3, h//2 - 10)), # Room 1 -> Corridor
        (area_ids[1][1], area_ids[4][1], (2*w//3, h//2 - 10), (2*w//3, h//2 - 10)), # Room 2 -> Corridor
        (area_ids[2][1], area_ids[4][1], (w//3, h//2 + 10), (w//3, h//2 + 10)), # Room 3 -> Corridor
        (area_ids[3][1], area_ids[4][1], (2*w//3, h//2 + 10), (2*w//3, h//2 + 10)), # Room 4 -> Corridor
        (area_ids[0][1], area_ids[1][1], (w//2, h//4), (w//2, h//4)), # Room 1 -> Room 2
    ]
    
    for from_area, to_area, p1, p2 in passages:
        pass_way_id = str(way_id_counter)
        way_id_counter -= 1
        
        lat1, lon1 = px_to_latlon(p1[0] - 2, p1[1])
        lat2, lon2 = px_to_latlon(p2[0] + 2, p2[1])
        
        n1 = str(node_id_counter)
        node_id_counter -= 1
        n2 = str(node_id_counter)
        node_id_counter -= 1
        
        ET.SubElement(root, "node", id=n1, action="modify", visible="true", lat=f"{lat1:.9f}", lon=f"{lon1:.9f}")
        ET.SubElement(root, "node", id=n2, action="modify", visible="true", lat=f"{lat2:.9f}", lon=f"{lon2:.9f}")
        
        pass_way = ET.SubElement(root, "way", id=pass_way_id, action="modify", visible="true")
        ET.SubElement(pass_way, "nd", ref=n1)
        ET.SubElement(pass_way, "nd", ref=n2)
        ET.SubElement(pass_way, "tag", k="osmAG:from", v=from_area)
        ET.SubElement(pass_way, "tag", k="osmAG:to", v=to_area)
        ET.SubElement(pass_way, "tag", k="osmAG:type", v="passage")
        ET.SubElement(pass_way, "tag", k="name", v=f"Door_{abs(int(pass_way_id))}")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    with open(output_path, "w") as f:
        f.write(xml_str)
    print(f"Successfully generated {output_path}")

if __name__ == "__main__":
    create_osmag_xml("docs/research/osm-AG/osm_ag_2/osm_ag_repo/osmAG/data/show/big_map_7.osm")
