#!/usr/bin/env python3
"""Create deterministic obstacle worlds and independent footprint-scoring maps.

The detour map is used ONLY for evaluation; navigation must keep the original
map and discover that obstacle through LiDAR. The blocked map is a known closure.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml


def prepare(workspace,contact_assets,output):
    output.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((workspace/'assets/gcu_ground_floor.yaml').read_text());r=cfg['resolution'];ox,oy=cfg['origin'][:2]
    source=np.flipud(np.asarray(Image.open(workspace/'assets'/cfg['image']))).copy()
    cases={'detour':{'center':[.95,-12.960313240266368],'size':[.45,.65,.6]},
           'blocked':{'center':[0.,-12.4],'size':[18.2,.25,.6]}}
    for name,item in cases.items():
        target=output/name;target.mkdir();world=ET.parse(contact_assets/'world.sdf');w=world.getroot().find('world')
        model=ET.SubElement(w,'model',name='test_obstacle_'+name);ET.SubElement(model,'static').text='true'
        x,y=item['center'];sx,sy,sz=item['size'];ET.SubElement(model,'pose').text=f'{x} {y} {sz/2} 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        for tag in ['collision','visual']:
            e=ET.SubElement(link,tag,name=tag);g=ET.SubElement(e,'geometry');b=ET.SubElement(g,'box');ET.SubElement(b,'size').text=f'{sx} {sy} {sz}'
        world.write(target/'world.sdf',encoding='utf-8',xml_declaration=True)
        grid=source.copy();rows,cols=np.indices(grid.shape)
        # Conservative overlay: every occupancy cell whose rectangle touches box.
        hit=(ox+cols*r<=x+sx/2)&(ox+(cols+1)*r>=x-sx/2)&(oy+rows*r<=y+sy/2)&(oy+(rows+1)*r>=y-sy/2)
        grid[hit]=0;Image.fromarray(np.flipud(grid)).save(target/'map.pgm')
        c=dict(cfg,image='map.pgm');(target/'map.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
        item.update(navigation_map='original' if name=='detour' else 'fixture',score_map=str(target/'map.yaml'))
    (output/'fixtures.json').write_text(json.dumps(cases,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--contact-assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();prepare(a.workspace,a.contact_assets,a.output)
