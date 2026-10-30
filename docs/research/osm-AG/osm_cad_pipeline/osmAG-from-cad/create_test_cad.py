import ezdxf

def create_floorplan_dxf(output_path):
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    
    # Layer for walls
    doc.layers.add("WALLS", color=7)
    # Layer for text
    doc.layers.add("TEXT", color=1)
    
    # Helper to add wall line
    def add_wall(p1, p2):
        msp.add_line(p1, p2, dxfattribs={"layer": "WALLS"})
        
    # Helper to add text
    def add_label(text, pos):
        msp.add_text(text, dxfattribs={"layer": "TEXT", "height": 0.5}).set_placement(pos)

    # 1. Outer Perimeter (24m x 16m)
    # Bottom wall: (0, 0) to (24, 0)
    add_wall((0, 0), (24, 0))
    # Top wall: (0, 16) to (24, 16)
    add_wall((0, 16), (24, 16))
    # Left wall: (0, 0) to (0, 16)
    add_wall((0, 0), (0, 16))
    # Right wall: (24, 0) to (24, 16)
    add_wall((24, 0), (24, 16))

    # 2. Corridor Walls (Horizontal hallway from y=6.5 to y=9.5)
    # North Corridor Wall at y=9.5 with 3 doors (gap 1.2m)
    # Door 1 at x=[3.4, 4.6]
    add_wall((0, 9.5), (3.4, 9.5))
    add_wall((4.6, 9.5), (8, 9.5))
    # Door 2 at x=[11.4, 12.6]
    add_wall((8, 9.5), (11.4, 9.5))
    add_wall((12.6, 9.5), (16, 9.5))
    # Door 3 at x=[19.4, 20.6]
    add_wall((16, 9.5), (19.4, 9.5))
    add_wall((20.6, 9.5), (24, 9.5))

    # South Corridor Wall at y=6.5 with 2 doors (gap 1.2m)
    # Door 4 at x=[5.4, 6.6]
    add_wall((0, 6.5), (5.4, 6.5))
    add_wall((6.6, 6.5), (12, 6.5))
    # Door 5 at x=[17.4, 18.6]
    add_wall((12, 6.5), (17.4, 6.5))
    add_wall((18.6, 6.5), (24, 6.5))

    # 3. Vertical Dividing Walls
    # North dividing walls (y from 9.5 to 16)
    add_wall((8, 9.5), (8, 16))
    add_wall((16, 9.5), (16, 16))
    
    # South dividing wall (y from 0 to 6.5)
    add_wall((12, 0), (12, 6.5))

    # 4. Text Labels
    add_label("Office_1", (3.0, 12.5))
    add_label("Office_2", (11.0, 12.5))
    add_label("Lab_Room", (19.0, 12.5))
    add_label("Conference", (4.5, 3.0))
    add_label("Office_3", (16.5, 3.0))
    add_label("Main_Corridor", (10.0, 8.0))

    doc.saveas(output_path)
    print(f"Generated test floor plan CAD DXF at: {output_path}")

if __name__ == "__main__":
    import os
    out = os.path.abspath("docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/test_floorplan.dxf")
    create_floorplan_dxf(out)
