#!/usr/bin/env python3
"""Motion-free ROS2 wrapper for our explicitly labelled paper-based planner."""
import argparse
import json
import math
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',required=True)
    args,rosargs=parser.parse_known_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped,Point,TransformStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from visualization_msgs.msg import Marker,MarkerArray
    from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster,TransformException
    from osmag_paper_planner import PaperPlanner,RouteError

    class Preview(Node):
        def __init__(self):
            super().__init__('gcu_osmag_route_preview',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.declare_parameter('base_frame','base_footprint')
            self.declare_parameter('disabled_passages',[''])
            self.declare_parameter('max_tf_age_s',.5)
            self.planner=PaperPlanner(args.assets)
            self.coordinate_audit=self.planner.coordinate_report()
            if not self.coordinate_audit['passed']:raise ValueError('coordinate_alignment_failed')
            self.instance=uuid.uuid4().hex[:8];self.request=0;self.clock_wall=None
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path_pub=self.create_publisher(Path,'/osmag/global_path',qos)
            self.graph_path_pub=self.create_publisher(Path,'/osmag/graph_path',qos)
            self.status_pub=self.create_publisher(String,'/osmag/route_status',qos)
            self.markers_pub=self.create_publisher(MarkerArray,'/osmag/map_markers',qos)
            self.input=self.create_subscription(PoseStamped,'/osmag/preview_goal',self.goal,10)
            sensor=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.clock_sub=self.create_subscription(Clock,'/clock',lambda _m:setattr(self,'clock_wall',time.monotonic()),sensor)
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            # The adapter outputs were checked against raster/world coordinates.
            # This alias does not own or replace the AMCL map -> odom transform.
            self.alias=StaticTransformBroadcaster(self)
            tf=TransformStamped();tf.header.frame_id='AGmap';tf.child_frame_id='map';tf.transform.rotation.w=1.
            self.alias.sendTransform(tf)
            self.marker_timer=self.create_timer(2.,self.publish_markers)
            self.readiness_timer=self.create_timer(.5,self.readiness)
            self.publish_markers();self.status('idle',ready=False)

        def readiness(self):
            if self.request:return
            ready=False
            if self.clock_wall is not None and time.monotonic()-self.clock_wall<=2.:
                try:
                    tf=self.buffer.lookup_transform('map',self.get_parameter('base_frame').value,Time())
                    age=(self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
                    ready=-.3<=age<=self.get_parameter('max_tf_age_s').value
                except TransformException:pass
            self.status('idle',ready=ready)

        def status(self,state,**fields):
            self.status_pub.publish(String(data=json.dumps({'state':state,'request':self.request,
                'route_id':self.instance+'-'+str(self.request),'motion_enabled':False,
                'backend':self.planner.backend,'map_sha256':self.planner.map_hash,**fields})))

        def clear(self):
            msg=Path();msg.header.frame_id='AGmap';msg.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(msg)
            self.graph_path_pub.publish(msg)

        def goal(self,msg):
            self.request+=1;self.clear()
            try:
                p=msg.pose.position;q=msg.pose.orientation
                if msg.header.frame_id!='map':raise RouteError('goal_frame_must_be_map')
                if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]):raise RouteError('nonfinite_goal')
                if abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1.)>.01:raise RouteError('invalid_goal_orientation')
                if abs(p.z)>.1:raise RouteError('unsupported_goal_height')
                if self.clock_wall is None or time.monotonic()-self.clock_wall>2.:raise RouteError('simulation_clock_not_fresh')
                try:tf=self.buffer.lookup_transform('map',self.get_parameter('base_frame').value,Time())
                except TransformException as error:raise RouteError('robot_tf_unavailable') from error
                age=(self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
                if age>self.get_parameter('max_tf_age_s').value or age<-.3:raise RouteError('robot_tf_stale')
                start=[tf.transform.translation.x,tf.transform.translation.y]
                disabled=[v for v in self.get_parameter('disabled_passages').value if v]
                result=self.planner.plan(start,[p.x,p.y],disabled)
                def message(points):
                    path=Path();path.header.frame_id='AGmap';path.header.stamp=self.get_clock().now().to_msg()
                    for i,(x,y) in enumerate(points):
                        pose=PoseStamped();pose.header=path.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                        if i==len(points)-1:pose.pose.orientation=q
                        else:
                            nx,ny=points[i+1];heading=math.atan2(ny-y,nx-x)
                            pose.pose.orientation.z=math.sin(heading/2);pose.pose.orientation.w=math.cos(heading/2)
                        path.poses.append(pose)
                    return path
                path=message(result['path']);graph_path=message(result['graph_path'])
                self.graph_path_pub.publish(graph_path)
                self.path_pub.publish(path)
                result={k:v for k,v in result.items() if k not in ['path','graph_path']}
                result.update(route_frame='AGmap',metric_frame='map',tf_age_s=age,poses=len(path.poses),
                              graph_poses=len(graph_path.poses),displayed_path='refined_geometry')
                self.status('ready',**result)
            except (RouteError,ValueError) as error:self.status('rejected',reason=str(error))
            except Exception as error:
                self.get_logger().error('Route preview failed: '+repr(error))
                self.status('failed',reason='internal_planner_error',detail=str(error))

        def publish_markers(self):
            output=MarkerArray()
            for i,(name,polygon) in enumerate(self.planner.areas.items()):
                marker=Marker();marker.header.frame_id='AGmap';marker.ns='osmAG_sectors';marker.id=i
                marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.scale.x=.025
                marker.color.r=.2;marker.color.g=.6;marker.color.b=1.;marker.color.a=.7;marker.pose.orientation.w=1.
                marker.points=[Point(x=float(x),y=float(y),z=.03) for x,y in polygon.exterior.coords]
                output.markers.append(marker)
            for i,item in enumerate(self.planner.interface_report['interfaces']):
                marker=Marker();marker.header.frame_id='AGmap';marker.ns='osmAG_interfaces';marker.id=i
                marker.type=Marker.LINE_LIST;marker.action=Marker.ADD;marker.scale.x=.07;marker.pose.orientation.w=1.
                marker.color.r=0. if item['execution_reference_feasible'] else 1.
                marker.color.g=1. if item['execution_reference_feasible'] else .1;marker.color.a=1.
                marker.points=[Point(x=float(x),y=float(y),z=.06) for x,y in self.planner.passage_segments[item['id']]]
                output.markers.append(marker)
            self.markers_pub.publish(output)

    rclpy.init(args=rosargs);node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
