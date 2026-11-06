"""Motion-free, footprint-constrained paper-based osmAG reference planner.

Our implementation of the published architecture, not the authors' unreleased
2026 runtime. Uses the existing GCU OSM convention and local interface audit.
No Euclidean fallback, synthetic start pose, velocity publisher or Nav2 client.
"""
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from pyproj import CRS,Transformer
from shapely.geometry import Point,Polygon,LineString
try:
    from shapely import contains_xy
except ImportError:
    from shapely.vectorized import contains as contains_xy
import yaml
from audit_gcu_interfaces import audit
from gcu_plan_validation import validate_path,cell_at
from osmag_hierarchy import HierarchyGraph
from osmag_route_refinement import RouteCorridor,refine_route


class RouteError(ValueError):
    pass


def length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


class PaperPlanner:
    backend='paper_based_reference_python_not_author_runtime'

    def __init__(self, assets, radius=.18):
        begin=time.perf_counter();self.assets=Path(assets);self.radius=radius
        cfg=yaml.safe_load((self.assets/'gcu_ground_floor.yaml').read_text())
        self.resolution=cfg['resolution'];self.origin=cfg['origin'][:2]
        if abs(cfg['origin'][2])>1e-10:raise ValueError('rotated_map_unsupported')
        image=np.flipud(np.asarray(Image.open(self.assets/cfg['image'])))
        self.grid=np.where(image==254,0,np.where(image==0,100,-1))
        free=np.pad((self.grid==0).astype(np.uint8),1)
        distance=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*self.resolution
        self.safe=distance>radius+self.resolution/math.sqrt(2)+self.resolution/8
        osm=ET.parse(self.assets/'gcu_ground_floor.osm').getroot();root=osm.find('node')
        crs=CRS.from_proj4(f"+proj=poly +lat_0={root.get('lat')} +lon_0={root.get('lon')} +ellps=WGS84 +units=m +no_defs")
        project=Transformer.from_crs('EPSG:4326',crs,always_xy=True)
        nodes={n.get('id'):project.transform(float(n.get('lon')),float(n.get('lat'))) for n in osm.findall('node')}
        self.areas={};self.parents={};self.passages={};self.passage_segments={};self.rasters={}
        for way in osm.findall('way'):
            tags={t.get('k'):t.get('v') for t in way.findall('tag')}
            if tags.get('osmAG:type')=='area':
                polygon=Polygon([nodes[n.get('ref')] for n in way.findall('nd')])
                if not polygon.is_valid:raise ValueError('invalid_area_polygon')
                name=tags['name'];self.areas[name]=polygon;self.parents[name]=tags.get('osmAG:parent')
            elif tags.get('osmAG:type')=='passage':
                self.passages[way.get('id')]=tags
                self.passage_segments[way.get('id')]=[nodes[n.get('ref')] for n in way.findall('nd')]
        containers={p for p in self.parents.values() if p is not None}
        self.leaves={n:p for n,p in self.areas.items() if n not in containers}
        if any(p not in self.areas for p in containers):raise ValueError('unknown_parent')
        self.points={};self.owners={};self.graph={};self.traces={};self.residents={n:[] for n in self.leaves}
        self.interface_report=audit(self.assets,radius)
        self.interface_report['input_sha256']=self.fingerprints()
        for passage in self.interface_report['interfaces']:
            ident=passage['id'];a,b=passage['areas'];passage['execution_reference_feasible']=False
            if not passage['sampled_local_crossing_found']:
                passage['exclusion_reason']='no_sampled_straight_crossing';continue
            if a not in self.leaves or b not in self.leaves:
                passage['exclusion_reason']='nonleaf_incident_area';continue
            crossing=self.center_crossing(ident,a,b)
            if crossing is None:
                passage['exclusion_reason']='no_conservative_raster_anchor';continue
            before,after=crossing
            first=a+'|'+ident;second=b+'|'+ident
            for key,area,point in [(first,a,before),(second,b,after)]:
                self.points[key]=tuple(point);self.owners[key]=area;self.graph[key]={};self.residents[area].append(key)
            self.edge(first,second,[before,after])
            passage['execution_reference_feasible']=True
            passage['directed_anchors']={a:before,b:after}
            passage['initial_audit_crossing']=passage['crossing']
            passage['crossing']={'point':[(before[0]+after[0])/2,(before[1]+after[1])/2],
                'crossing':[before,after],
                'validation':validate_path(self.grid,self.resolution,self.origin,[before,after],self.radius)}
            passage['anchor_policy']='nearest interface-midpoint sample with validated crossing and conservative raster connectors'
        self.local_edge_failures=[]
        self.redundant_interface_pairs=[]
        for area,residents in self.residents.items():
            for i,a in enumerate(residents):
                for b in residents[i+1:]:
                    one=self.passages[a.split('|')[-1]];two=self.passages[b.split('|')[-1]]
                    if {one['osmAG:from'],one['osmAG:to']}=={two['osmAG:from'],two['osmAG:to']}:
                        self.redundant_interface_pairs.append({'area':area,'states':[a,b]})
                        continue
                    path=self.grid_path(area,self.points[a],self.points[b])
                    if path is not None:self.edge(a,b,path)
                    else:self.local_edge_failures.append({'area':area,'passages':[a.split('|')[-1],b.split('|')[-1]]})
        self.hierarchy=HierarchyGraph(self.points,self.graph,self.owners,self.parents)
        self.preprocess_s=time.perf_counter()-begin
        self.map_hash=hashlib.sha256(json.dumps(self.fingerprints(),sort_keys=True).encode()).hexdigest()

    def fingerprints(self):
        return {n:hashlib.sha256((self.assets/n).read_bytes()).hexdigest()
                for n in ['gcu_ground_floor.osm','gcu_ground_floor.pgm','gcu_ground_floor.yaml']}

    def edge(self,a,b,path):
        cost=length(path);self.graph[a][b]=cost;self.graph[b][a]=cost
        self.traces[a,b]=path;self.traces[b,a]=path[::-1]

    def center_crossing(self, ident, first, second):
        a,b=self.passage_segments[ident];distance=math.dist(a,b)
        normal=(-(b[1]-a[1])/distance,(b[0]-a[0])/distance)
        samples=np.linspace(0,1,max(3,math.ceil(distance/(self.resolution/4))+1))
        for t in sorted(set([.5,*samples]),key=lambda t:abs(t-.5)):
            c=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]));offset=self.radius+self.resolution
            before=[c[0]-offset*normal[0],c[1]-offset*normal[1]]
            after=[c[0]+offset*normal[0],c[1]+offset*normal[1]]
            if not self.leaves[first].covers(Point(before)):before,after=after,before
            if not (self.leaves[first].covers(Point(before)) and self.leaves[second].covers(Point(after))):continue
            if not validate_path(self.grid,self.resolution,self.origin,[before,after],self.radius)['valid']:continue
            if self.connector(first,before) is not None and self.connector(second,after) is not None:return before,after
        return None

    def raster(self, area):
        if area not in self.rasters:
            polygon=self.leaves[area];x0,y0,x1,y1=polygon.bounds;r=self.resolution;ox,oy=self.origin
            c0=max(0,math.floor((x0-ox)/r)-1);c1=min(self.grid.shape[1],math.ceil((x1-ox)/r)+1)
            r0=max(0,math.floor((y0-oy)/r)-1);r1=min(self.grid.shape[0],math.ceil((y1-oy)/r)+1)
            rows,cols=np.mgrid[r0:r1,c0:c1]
            mask=self.safe[r0:r1,c0:c1]&contains_xy(polygon,ox+(cols+.5)*r,oy+(rows+.5)*r)
            self.rasters[area]=(mask,r0,c0)
        return self.rasters[area]

    def xy(self, row, col):
        return [self.origin[0]+(col+.5)*self.resolution,self.origin[1]+(row+.5)*self.resolution]

    def connector(self, area, point):
        mask,r0,c0=self.raster(area);r=self.resolution
        row=math.floor((point[1]-self.origin[1])/r)-r0;col=math.floor((point[0]-self.origin[0])/r)-c0
        candidates=[]
        for y in range(max(0,row-4),min(mask.shape[0],row+5)):
            for x in range(max(0,col-4),min(mask.shape[1],col+5)):
                if mask[y,x]:candidates.append((math.dist(point,self.xy(y+r0,x+c0)),y,x))
        for _,y,x in sorted(candidates):
            center=self.xy(y+r0,x+c0)
            contained=math.dist(point,center)<1e-10 or self.leaves[area].covers(LineString([point,center]))
            if contained and validate_path(self.grid,r,self.origin,[point,center],self.radius)['valid']:return (y,x)
        return None

    def grid_path(self, area, start, goal):
        first=self.connector(area,start);last=self.connector(area,goal)
        if first is None or last is None:return None
        mask,r0,c0=self.raster(area);distances={first:0.};previous={};counter=itertools.count()
        queue=[(0.,next(counter),0.,first)]
        while queue:
            _,_,cost,current=heapq.heappop(queue)
            if cost!=distances[current]:continue
            if current==last:break
            y,x=current
            for dy,dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                ny,nx=y+dy,x+dx
                if not (0<=ny<mask.shape[0] and 0<=nx<mask.shape[1]) or not mask[ny,nx]:continue
                if dy and dx and not (mask[y,nx] and mask[ny,x]):continue
                next_cost=cost+math.hypot(dx,dy);neighbor=(ny,nx)
                if next_cost<distances.get(neighbor,math.inf)-1e-12:
                    distances[neighbor]=next_cost;previous[neighbor]=current
                    heapq.heappush(queue,(next_cost+math.dist(neighbor,last),next(counter),next_cost,neighbor))
        if last not in distances:return None
        walk=[last]
        while walk[-1]!=first:walk.append(previous[walk[-1]])
        path=[list(start)]+[self.xy(y+r0,x+c0) for y,x in walk[::-1]]+[list(goal)]
        if not self.leaves[area].covers(LineString(path)):return None
        if not validate_path(self.grid,self.resolution,self.origin,path,self.radius)['valid']:return None
        return path

    def containing_area(self, point, role):
        if not all(math.isfinite(v) for v in point):raise RouteError('nonfinite_'+role)
        value=cell_at(self.grid,self.resolution,self.origin,*point)
        if value!=0:raise RouteError('outside_'+role if value is None else 'unknown_'+role if value<0 else 'occupied_'+role)
        if not validate_path(self.grid,self.resolution,self.origin,[point],self.radius)['valid']:raise RouteError(role+'_footprint_blocked')
        found=[n for n,p in self.leaves.items() if p.contains(Point(point))]
        if len(found)!=1:raise RouteError('ambiguous_or_missing_'+role+'_sector')
        return found[0]

    def plan(self, start, goal, disabled_passages=()):
        begin=time.perf_counter();start=list(start[:2]);goal=list(goal[:2])
        first=self.containing_area(start,'start');last=self.containing_area(goal,'goal')
        disabled=set(map(str,disabled_passages))
        if not disabled<=set(self.passages):raise RouteError('unknown_disabled_passage')
        if first==last:
            path=self.grid_path(first,start,goal)
            if path is None:raise RouteError('no_feasible_local_route')
            result={'graph_cost_m':length(path),'passage_ids':[],'area_ids':[first],'lca':first,
                    'compact_search_nodes':0,'base_nodes':len(self.points),'method':'footprint_checked_leaf_raster_astar'}
        else:
            def attach(area,point):
                paths={}
                for node in self.residents[area]:
                    if node.split('|')[-1] in disabled:continue
                    route=self.grid_path(area,point,self.points[node])
                    if route is not None:paths[node]=route
                return paths
            starts=attach(first,start);goals=attach(last,goal)
            blocked={n for n in self.points if n.split('|')[-1] in disabled}
            try:route=self.hierarchy.route(first,last,{n:length(p) for n,p in starts.items()},
                    {n:length(p) for n,p in goals.items()},start,goal,blocked)
            except ValueError as error:raise RouteError(str(error)) from error
            nodes=route.pop('nodes');path=list(starts[nodes[0]])
            for a,b in zip(nodes,nodes[1:]):path.extend(self.traces[a,b][1:])
            path.extend(goals[nodes[-1]][::-1][1:]);areas=[first];passages=[]
            for a,b in zip(nodes,nodes[1:]):
                if self.owners[a]!=self.owners[b]:areas.append(self.owners[b]);passages.append(a.split('|')[-1])
            cost=route.pop('cost_m')
            result={**route,'graph_cost_m':cost,'passage_ids':passages,'area_ids':areas}
        validation=validate_path(self.grid,self.resolution,self.origin,path,self.radius)
        if not validation['valid']:raise RouteError('expanded_path_footprint_blocked')
        if abs(length(path)-result['graph_cost_m'])>1e-7:raise RouteError('expanded_cost_mismatch')
        graph_path=path
        corridor=RouteCorridor(self.leaves,self.passage_segments,result['area_ids'],result['passage_ids'],disabled)
        physical=lambda points:validate_path(self.grid,self.resolution,self.origin,points,self.radius)
        try:path,refinement=refine_route(graph_path,corridor,physical)
        except ValueError as error:raise RouteError(str(error)) from error
        refined_validation=physical(path)
        if not refined_validation['valid']:raise RouteError('refined_path_footprint_blocked')
        result.update(backend=self.backend,planner_version='phase5_preview_v2',path=path,graph_path=graph_path,
                      refined_length_m=length(path),refinement=refinement,start=start,goal=goal,frame='map',
                      map_sha256=self.map_hash,disabled_passages=sorted(disabled),validation=refined_validation,
                      graph_validation=validation,
                      preprocess_s=self.preprocess_s,query_s=time.perf_counter()-begin,
                      timing_scope='query includes endpoint searches, hierarchy query and refinement; new closure summaries are built on first use')
        return result

    def coordinate_report(self):
        features=json.loads((self.assets/'regions_pixels.geojson').read_text())['features']
        readiness=json.loads((self.assets/'readiness_report.json').read_text());spawn=readiness['spawn'];r=self.resolution
        refx=spawn['pixel'][0]-spawn['x']/r;refy=spawn['pixel'][1]+spawn['y']/r
        pairs=[];errors=[]
        for feature in features:
            name=feature['properties']['name'];polygon=self.areas[name]
            for x,y in feature['geometry']['coordinates'][0]:
                expected=((x-refx)*r,(refy-y)*r)
                actual=min(polygon.exterior.coords,key=lambda v:math.dist(v,expected))
                pairs.append((expected,actual));errors.append(math.dist(expected,actual))
        a=pairs[0];b=max(pairs,key=lambda p:math.dist(a[0],p[0]))
        def cross(p):return abs((b[0][0]-a[0][0])*(p[0][1]-a[0][1])-(b[0][1]-a[0][1])*(p[0][0]-a[0][0]))
        c=max(pairs,key=cross)
        if cross(c)<1.:raise ValueError('landmarks_not_nonlinear')
        return {'passed':max(errors)<=r,'checked_vertices':len(errors),'maximum_error_m':max(errors),
                'tolerance_m':r,'landmarks':[{'raster_world':p[0],'projected_osm':p[1],'error_m':math.dist(*p)} for p in [a,b,c]],
                'map_origin':self.origin,'resolution':r,'units':'metres','floor':'ground',
                'semantic_to_metric_transform':'identity; verified file-coordinate convention',
                'limitations':'Consistency of independently serialized OSM/raster adapter outputs; not an external survey, physical calibration or live TF ownership test.'}
