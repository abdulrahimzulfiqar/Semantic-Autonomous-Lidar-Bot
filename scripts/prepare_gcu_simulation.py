#!/usr/bin/env python3
"""Prepare canonical simple-way osmAG and aligned Gazebo/ROS map artifacts.

Run with the CAD pipeline venv. Never infer collision walls from region borders.
All generated coordinates share the CAD export's local root frame.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image, ImageDraw
import yaml
from shapely.geometry import Polygon, LineString, Point, mapping
from shapely.ops import split
from shapely import union_all

ROOT = Path(__file__).resolve().parents[1]
CAD = ROOT / 'docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad'
sys.path.insert(0, str(CAD))
from reconstruct_regions import export_osm, mask_geometry, polygonal


def simple_parts(polygon):
    """Partition through holes; do not fill them or approximate them away."""
    pending, result = [polygon], []
    while pending:
        current = pending.pop()
        if not current.interiors:
            result.append(current.simplify(0, preserve_topology=True))
            continue
        hole = Polygon(current.interiors[0]).representative_point()
        xmin, ymin, xmax, ymax = current.bounds
        parts = list(split(current, LineString([(hole.x, ymin-1), (hole.x, ymax+1)])).geoms)
        if len(parts) < 2 or sum(len(p.interiors) for p in parts) >= len(current.interiors):
            raise ValueError('Cannot partition hole safely')
        pending.extend(parts)
    return sorted(result, key=lambda p: (*p.bounds, p.area))


def lines(geometry):
    if geometry.geom_type == 'LineString':
        return [geometry] if geometry.length > 1e-8 else []
    return [line for g in getattr(geometry, 'geoms', []) for line in lines(g)]


def straight_span(line):
    coords = list(line.simplify(0).coords)
    a, b = max(zip(coords, coords[1:]), key=lambda pair: Point(pair[0]).distance(Point(pair[1])))
    return LineString([a, b])


def node_ring(polygon, endpoints):
    coords = list(polygon.simplify(0, preserve_topology=True).exterior.coords)
    result = []
    for a, b in zip(coords, coords[1:]):
        result.append(a)
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        candidates = []
        for point in endpoints:
            t = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2
            if 1e-10 < t < 1-1e-10 and LineString([a,b]).distance(Point(point)) < 1e-8:
                candidates.append((t,point))
        result.extend(point for _,point in sorted(set(candidates)))
    return Polygon(result)


def rectangles(mask):
    """Coalesce horizontal runs into exact pixel-edge collision boxes."""
    active = {}
    for y in range(mask.shape[0]+1):
        changes = np.flatnonzero(np.diff(np.pad(mask[y].astype(np.int8),(1,1)))) if y < mask.shape[0] else []
        runs = set(zip(map(int,changes[::2]),map(int,changes[1::2])))
        for run in sorted(set(active)-runs):
            yield run[0]-.5, active.pop(run)-.5, run[1]-.5, y-.5
        for run in sorted(runs-set(active)): active[run] = y


def world_sdf(walls, resolution, reference, height, api, render_engine='ogre2'):
    sdf = ET.Element('sdf',version='1.8');world=ET.SubElement(sdf,'world',name='gcu_ground_floor')
    physics=ET.SubElement(world,'physics',name='physics',type='ignored')
    ET.SubElement(physics,'max_step_size').text='0.001';ET.SubElement(physics,'real_time_factor').text='1'
    prefix, namespace = ('ignition-gazebo','ignition::gazebo') if api=='fortress' else ('gz-sim','gz::sim')
    for suffix, name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        plugin=ET.SubElement(world,'plugin',filename=f'{prefix}-{suffix}-system',name=f'{namespace}::systems::{name}')
        if name=='Sensors': ET.SubElement(plugin,'render_engine').text=render_engine
    light=ET.SubElement(world,'light',name='sun',type='directional');ET.SubElement(light,'pose').text='0 0 20 0 0 0'
    ET.SubElement(light,'diffuse').text='0.8 0.8 0.8 1';ET.SubElement(light,'direction').text='-0.5 0.1 -0.9'
    ground=ET.SubElement(world,'model',name='ground');ET.SubElement(ground,'static').text='true';link=ET.SubElement(ground,'link',name='ground')
    for kind in ('collision','visual'):
        element=ET.SubElement(link,kind,name=kind);geom=ET.SubElement(element,'geometry');plane=ET.SubElement(geom,'plane')
        ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='200 200'
    model=ET.SubElement(world,'model',name='cad_walls');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='walls')
    boxes=list(rectangles(walls))
    for i,(xmin,ymin,xmax,ymax) in enumerate(boxes):
        x=((xmin+xmax)/2-reference['pixel_x'])*resolution;y=(reference['pixel_y']-(ymin+ymax)/2)*resolution
        for kind in ('collision','visual'):
            item=ET.SubElement(link,kind,name=f'{kind}_{i}');ET.SubElement(item,'pose').text=f'{x:.12f} {y:.12f} {height/2} 0 0 0'
            box=ET.SubElement(ET.SubElement(item,'geometry'),'box')
            ET.SubElement(box,'size').text=f'{(xmax-xmin)*resolution:.12f} {(ymax-ymin)*resolution:.12f} {height}'
    ET.indent(sdf)
    return ET.ElementTree(sdf),boxes


def prepare(manifest_path, output, gazebo_api='fortress', wall_height=2.0, robot_radius=.18, render_engine='ogre2'):
    manifest=json.loads(manifest_path.read_text());out=manifest['outputs']
    def source(key):
        path=Path(out[key]);return path if path.is_absolute() else CAD/path
    effective=Path(manifest['effective_config']);effective=effective if effective.is_absolute() else CAD/effective
    config=yaml.safe_load(effective.read_text());resolution=float(config['map_preprocessing']['resolution']);reference=config['root_node']
    original=json.loads(source('reconstructed_regions').read_text())['features'];regions={};types={};parents={};source_names={}
    index=1
    for feature in sorted(original,key=lambda f:int(f['properties']['name'].split('_')[-1])):
        from shapely.geometry import shape
        original_geometry=shape(feature['geometry']);parts=[part for p in polygonal(original_geometry) for part in simple_parts(p)]
        for part in parts:
            regions[index]={'geometry':part};source_names[index]=feature['properties']['name']
            types[index]='corridor' if feature['properties']['name']=='room_1' else 'room';index+=1
        assert abs(union_all(parts).symmetric_difference(original_geometry).area)<1e-6
    passages=[]
    for a in regions:
        for b in regions:
            if b<=a:continue
            shared=regions[a]['geometry'].boundary.intersection(regions[b]['geometry'].boundary)
            passages.extend(((a,b),straight_span(line)) for line in lines(shared))
    endpoints={point for pair,line in passages for point in line.coords}
    for region in regions.values():region['geometry']=node_ring(region['geometry'],endpoints)
    envelope=np.asarray(Image.open(source('area_graph_dir')/'building_envelope.png').convert('L'))>127
    floor=Polygon(max(polygonal(mask_geometry(envelope)),key=lambda p:p.area).exterior)
    assert all(floor.buffer(1e-8).covers(r['geometry']) for r in regions.values())
    floor_index=index;regions[floor_index]={'geometry':floor};types[floor_index]='structure'
    output.mkdir(parents=True,exist_ok=True)
    osm_path=output/'gcu_ground_floor.osm';export_osm(osm_path,regions,passages,config)
    tree=ET.parse(osm_path);osm=tree.getroot();names={f'room_{i}':('gcu_gf_floor' if i==floor_index else f'gcu_gf_{types[i]}_{i}') for i in regions}
    for element in osm:
        tags={t.attrib['k']:t for t in element.findall('tag')};old=tags.get('name')
        if old is not None and old.attrib['v'] in names:
            i=int(old.attrib['v'].split('_')[-1]);old.attrib['v']=names[f'room_{i}'];tags['osmAG:areaType'].attrib['v']=types[i]
            if i!=floor_index:
                ET.SubElement(element,'tag',k='osmAG:parent',v='gcu_gf_floor')
                ET.SubElement(element,'tag',k='source:area',v=source_names[i])
        elif old is not None and old.attrib['v'].startswith('p_'):old.attrib['v']='gcu_gf_'+old.attrib['v']
        if 'height' in tags:
            tags['height'].attrib['v']='0'  # This single simulated floor is at world z=0.
        for key in ('osmAG:from','osmAG:to'):
            if key in tags:tags[key].attrib['v']=names[tags[key].attrib['v']]
    ET.indent(osm);tree.write(osm_path,encoding='utf-8',xml_declaration=True)
    nodes={n.attrib['id'] for n in osm.findall('node')};area_refs={};way_names={}
    for way in osm.findall('way'):
        tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')};refs=[n.attrib['ref'] for n in way.findall('nd')]
        assert set(refs)<=nodes
        if tags.get('osmAG:type')=='area':area_refs[tags['name']]=refs;way_names[tags['name']]=way.attrib['id']
    for way in osm.findall('way'):
        tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')};refs=[n.attrib['ref'] for n in way.findall('nd')]
        if tags.get('osmAG:type')=='passage':
            assert len(refs)==2
            for name in (tags['osmAG:from'],tags['osmAG:to']):
                ring=area_refs[name];assert any({a,b}==set(refs) for a,b in zip(ring,ring[1:])),(name,refs)
    # Also supply numeric references for the older engine while the canonical
    # named-reference loader changes are being verified on ROS/Linux.
    legacy=copy.deepcopy(osm)
    for t in legacy.iter('tag'):
        if t.attrib['k'] in ('osmAG:from','osmAG:to','osmAG:parent'):t.attrib['v']=way_names[t.attrib['v']]
    ET.ElementTree(legacy).write(output/'gcu_ground_floor_legacy.osm',encoding='utf-8',xml_declaration=True)
    occupancy=np.asarray(Image.open(source('png')).convert('L'));wall_mask=occupancy<=127
    world,boxes=world_sdf(wall_mask,resolution,reference,wall_height,gazebo_api,render_engine);world.write(output/'gcu_ground_floor.sdf',encoding='utf-8',xml_declaration=True)
    # Exclude unassigned/exterior space in Nav2 even where it is physically empty.
    labels=np.load(source('reconstruction_labels'));map_image=np.full(labels.shape,205,np.uint8);map_image[wall_mask]=0;map_image[labels>0]=254
    Image.fromarray(map_image).save(output/'gcu_ground_floor.pgm')
    origin=[(-.5-reference['pixel_x'])*resolution,(reference['pixel_y']-(labels.shape[0]-.5))*resolution,0.0]
    (output/'gcu_ground_floor.yaml').write_text(yaml.safe_dump({'image':'gcu_ground_floor.pgm','mode':'trinary','resolution':resolution,'origin':origin,'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
    # Choose a high-clearance corridor spawn, not the root (which is in the void).
    clearance=cv2.distanceTransform((labels==1).astype('uint8'),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    y,x=np.unravel_index(clearance.argmax(),clearance.shape)
    spawn={'x':float((x-reference['pixel_x'])*resolution),'y':float((reference['pixel_y']-y)*resolution),'z':.15,'yaw':0.0,'pixel':[int(x),int(y)]}
    physical_clearance=cv2.distanceTransform((~wall_mask).astype('uint8'),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    interface_clearances=[]
    for _,line in passages:
        point=line.interpolate(.5,normalized=True)
        row,col=int(round(point.y)),int(round(point.x))
        interface_clearances.append(float(physical_clearance[row,col]))
    adjacent={i:set() for i in regions if i!=floor_index}
    for (a,b),_ in passages:adjacent[a].add(b);adjacent[b].add(a)
    visited={next(iter(adjacent))};pending=list(visited)
    while pending:
        for other in adjacent[pending.pop()]-visited:visited.add(other);pending.append(other)
    if visited!=set(adjacent):raise ValueError('Adapted passage topology is disconnected')
    preview=Image.new('RGB',(occupancy.shape[1],occupancy.shape[0]+35),'white')
    preview.paste(Image.fromarray(occupancy).convert('RGB'),(0,35));draw=ImageDraw.Draw(preview)
    draw.text((8,8),'Blue: virtual partitions | Red: interfaces | Green: spawn | Black: physical walls',fill='black')
    for i,region in regions.items():
        if i!=floor_index:draw.line([(x,y+35) for x,y in region['geometry'].exterior.coords],fill='#0086b3',width=1)
    for _,line in passages:
        point=line.interpolate(.5,normalized=True);x,y=point.x,point.y+35
        draw.ellipse((x-2,y-2,x+2,y+2),fill='red')
    x,y=spawn['pixel'][0],spawn['pixel'][1]+35;draw.ellipse((x-5,y-5,x+5,y+5),fill='#00bb33')
    preview.save(output/'alignment_preview.png')
    report={'topology_connected':True,'floor_elevation_m':0,'leaf_areas':len(regions)-1,'structure_areas':1,'two_node_passages':len(passages),'holes_preserved_by_partition':True,'shared_node_validation':'passed','walls_from_occupancy':True,'wall_boxes':len(boxes),'wall_height_assumption_m':wall_height,'map_origin':origin,'resolution':resolution,'spawn':spawn,'gazebo_api':gazebo_api,'render_engine':render_engine,'runtime_tested':False,'min_interface_midpoint_clearance_m':min(interface_clearances),'min_exported_interface_length_m':min(line.length*resolution for _,line in passages),'footprint_feasibility_verified':False,'reference_robot_radius_m':robot_radius,'interfaces_below_reference_radius':[{'areas':[names[f'room_{pair[0]}'],names[f'room_{pair[1]}']],'midpoint_clearance_m':clearance} for (pair,line),clearance in zip(passages,interface_clearances) if clearance<robot_radius],'wall_mask_sha256':hashlib.sha256(wall_mask.tobytes()).hexdigest(),'note':'Static artifacts validated; actual ROS loader, Gazebo physics and navigation must still be tested on Linux.'}
    (output/'readiness_report.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'regions_pixels.geojson').write_text(json.dumps({'type':'FeatureCollection','features':[{'type':'Feature','properties':{'name':names[f'room_{i}'],'type':types[i]},'geometry':mapping(r['geometry'])} for i,r in regions.items()]})+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=CAD/'runs/local-cad/gcu_config_parameters_testing_12/run_manifest.json')
    parser.add_argument('--output',type=Path,default=ROOT/'simulation/gcu_ground_floor')
    parser.add_argument('--render-engine',choices=['ogre','ogre2'],default='ogre2')
    parser.add_argument('--gazebo-api',choices=['fortress','gz'],default='fortress')
    parser.add_argument('--wall-height',type=float,default=2.)
    parser.add_argument('--robot-radius',type=float,default=.18,help='Clearance reference from current Nav2 config; does not certify full-path feasibility')
    args=parser.parse_args()
    if args.wall_height<=0 or args.robot_radius<=0:parser.error('Wall height and radius must be positive')
    print(json.dumps(prepare(args.manifest,args.output,args.gazebo_api,args.wall_height,args.robot_radius,args.render_engine),indent=2))

if __name__=='__main__':main()
