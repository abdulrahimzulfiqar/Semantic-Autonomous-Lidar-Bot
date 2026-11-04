#!/usr/bin/env python3
"""Expand a corrected evaluation robot and instrument every SDF collision.

Run in a sourced Ubuntu Humble/Fortress environment. Output must be new. No
processes are launched and no existing robot installation is changed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml


def prepare(workspace, output):
    output.mkdir(parents=True,exist_ok=False)
    urdf = subprocess.check_output(['xacro',str(workspace/'robot_profile/gcu_evaluation.urdf.xacro')])
    (output/'robot.urdf').write_bytes(urdf)
    converted = subprocess.check_output(['ign','sdf','-p',str(output/'robot.urdf')])
    robot = ET.fromstring(converted)
    entries=[];collisions=[]
    for link in robot.findall('./model/link'):
        for collision in link.findall('collision'):
            i=len(collisions);topic='/gcu/contacts/collision_'+str(i)
            sensor=ET.SubElement(link,'sensor',name='gcu_contact_'+str(i),type='contact')
            ET.SubElement(sensor,'always_on').text='true'
            ET.SubElement(sensor,'update_rate').text='20'
            ET.SubElement(sensor,'topic').text=topic
            contact=ET.SubElement(sensor,'contact')
            ET.SubElement(contact,'collision').text=collision.attrib['name']
            # Fortress contact system also accepts the contact-local topic.
            ET.SubElement(contact,'topic').text=topic
            collisions.append({'link':link.attrib['name'],'collision':collision.attrib['name'],'topic':topic})
            entries.append({'ros_topic_name':topic,'gz_topic_name':topic,
                'ros_type_name':'ros_gz_interfaces/msg/Contacts','gz_type_name':'gz.msgs.Contacts','direction':'GZ_TO_ROS'})
    if len(collisions)!=13:raise ValueError('Collision contract changed: expected 13, got '+str(len(collisions)))
    ET.ElementTree(robot).write(output/'robot.sdf',encoding='utf-8',xml_declaration=True)
    (output/'contact_bridge.yaml').write_text(yaml.safe_dump(entries,sort_keys=False))
    world=ET.parse(workspace/'assets/gcu_ground_floor_ogre.sdf')
    ET.SubElement(world.getroot().find('world'),'plugin',filename='ignition-gazebo-contact-system',
                  name='ignition::gazebo::systems::Contact')
    world.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    # Separate positive control: robot settles on a raised static box at spawn.
    w=world.getroot().find('world')
    fixture=ET.SubElement(w,'model',name='validation_obstacle')
    ET.SubElement(fixture,'static').text='true'
    ET.SubElement(fixture,'pose').text='-0.05864395131342248 -12.960313240266368 0.04 0 0 0'
    link=ET.SubElement(fixture,'link',name='link')
    for tag in ['collision','visual']:
        e=ET.SubElement(link,tag,name=tag);g=ET.SubElement(e,'geometry');b=ET.SubElement(g,'box')
        ET.SubElement(b,'size').text='0.6 0.6 0.08'
    world.write(output/'positive_control.sdf',encoding='utf-8',xml_declaration=True)
    manifest={'collisions':collisions,'files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in output.iterdir() if p.is_file()}}
    (output/'contact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);a=p.parse_args();prepare(a.workspace,a.output)
