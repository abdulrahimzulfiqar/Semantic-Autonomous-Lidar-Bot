#!/usr/bin/env python3
"""Static local swept-circle crossings of osmAG interfaces, not route certification.

Tests straight normal crossings between the two stated incident polygons at
sampled interface positions. Includes occupied/unknown cells. A failed local
search does not prove every curved route impossible. Run with the CAD venv.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from pyproj import CRS,Transformer
from shapely.geometry import Polygon,Point
import yaml
from gcu_plan_validation import validate_path,clearance


def audit(assets,radius=.18):
    config=yaml.safe_load((assets/'gcu_ground_floor.yaml').read_text());r=config['resolution'];origin=config['origin']
    image=np.flipud(np.asarray(Image.open(assets/config['image'])))
    grid=np.where(image==254,0,np.where(image==0,100,-1))
    osm=ET.parse(assets/'gcu_ground_floor.osm').getroot();root=osm.find('node')
    crs=CRS.from_proj4(f"+proj=poly +lat_0={root.attrib['lat']} +lon_0={root.attrib['lon']} +ellps=WGS84 +units=m +no_defs")
    transform=Transformer.from_crs('EPSG:4326',crs,always_xy=True)
    nodes={n.attrib['id']:transform.transform(float(n.attrib['lon']),float(n.attrib['lat'])) for n in osm.findall('node')}
    areas={};passages=[]
    for way in osm.findall('way'):
        tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')};pts=[nodes[n.attrib['ref']] for n in way.findall('nd')]
        if tags.get('osmAG:type')=='area':areas[tags['name']]=Polygon(pts)
        if tags.get('osmAG:type')=='passage':passages.append((way.attrib['id'],tags,pts))
    results=[]
    for ident,tags,(a,b) in passages:
        length=math.dist(a,b);normal=(-(b[1]-a[1])/length,(b[0]-a[0])/length)
        best=None;accepted=None
        for t in np.linspace(0,1,max(3,math.ceil(length/(r/4))+1)):
            c=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]));dist=clearance(grid,r,origin,*c,radius+2*r)
            if best is None or dist>best[0]:best=(dist,c)
            if dist<radius+r/8:continue
            before=(c[0]-normal[0]*(radius+r),c[1]-normal[1]*(radius+r));after=(c[0]+normal[0]*(radius+r),c[1]+normal[1]*(radius+r))
            left=areas[tags['osmAG:from']];right=areas[tags['osmAG:to']]
            incident=(left.covers(Point(before)) and right.covers(Point(after))) or (left.covers(Point(after)) and right.covers(Point(before)))
            if not incident:continue
            v=validate_path(grid,r,origin,[before,after],radius)
            if v['valid']:accepted={'point':c,'crossing':[before,after],'validation':v};break
        midpoint=((a[0]+b[0])/2,(a[1]+b[1])/2)
        results.append({'id':ident,'areas':[tags['osmAG:from'],tags['osmAG:to']],'length_m':length,
            'midpoint_clearance_lower_bound_m':clearance(grid,r,origin,*midpoint,radius+2*r),
            'sampled_local_crossing_found':accepted is not None,'crossing':accepted,
            'best_sample_clearance_lower_bound_m':best[0],'best_point':best[1]})
    goals={}
    for name,pt in [('corridor',[1.4413560486865775,-12.960313240266368]),('turn',[-3.,-10.5]),('room',[-10.,-7.5])]:
        goals[name]={'point':pt,'areas':[n for n,p in areas.items() if p.covers(Point(pt)) and n!='gcu_gf_floor'],
            'footprint':validate_path(grid,r,origin,[pt],radius)}
    return {'radius_m':radius,'sampling_m':r/4,'scope':'sampled straight local crossings only; no full route, turning, live-costmap or contact certification',
        'total':len(results),'local_crossings_found':sum(p['sampled_local_crossing_found'] for p in results),'interfaces':results,'test_goals':goals}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,default=Path('simulation/gcu_ground_floor'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(audit(a.assets),indent=2)+'\n')
