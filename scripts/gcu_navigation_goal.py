#!/usr/bin/env python3
"""Explicit simulation click-to-NavigateToPose bridge with validated preview.

/gcu/navigation_goal -> static footprint validation -> ComputePathToPose ->
full-path validation -> NavigateToPose. Nav2 owns motion and live replanning.
Busy clicks are rejected; /gcu/navigation_cancel (std_msgs/Empty) cancels the
active action. Ground truth is neither subscribed nor used by this node.
"""
import argparse
import time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from gcu_goal_preview import Preview


class NavigationGoal(Preview):
    def __init__(self):
        self.nav_goal_handle=None;self.sending=False
        super().__init__(prefix='/gcu/navigation',plan_only=False)
        self.navigation=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.cancel_sub=self.create_subscription(Empty,'/gcu/navigation_cancel',self.cancel,10)

    def goal_callback(self,goal):
        if self.pending or self.sending or self.nav_goal_handle is not None:
            self.status('busy',reason='active_goal_unchanged_cancel_before_new_goal');return
        super().goal_callback(goal)

    def validated(self,path,fields):
        if not self.navigation.server_is_ready():self.clear();self.status('rejected',reason='navigator_not_ready');return
        self.path_pub.publish(path);self.status('sending',**fields)
        action=NavigateToPose.Goal();action.pose=self.current_goal
        self.sending=True;self.send_deadline=time.monotonic()+15;request=self.generation
        self.navigation.send_goal_async(action).add_done_callback(lambda f:self.navigation_accepted(f,request))

    def navigation_accepted(self,future,request):
        try:handle=future.result()
        except Exception as e:
            if request==self.generation:
                self.sending=False;self.clear();self.status('rejected',reason='navigation_request_error',detail=str(e))
            return
        if request!=self.generation:
            if handle.accepted:handle.cancel_goal_async()
            return
        self.sending=False
        if not handle.accepted:self.clear();self.status('rejected',reason='navigation_action_rejected');return
        self.nav_goal_handle=handle;self.status('executing')
        handle.get_result_async().add_done_callback(lambda f:self.navigation_finished(f,request))

    def navigation_finished(self,future,request):
        if request!=self.generation:return
        self.nav_goal_handle=None
        try:result=future.result()
        except Exception as e:self.clear();self.status('failed',detail=str(e));return
        state={4:'succeeded',5:'cancelled',6:'failed'}.get(result.status,'failed')
        if state!='succeeded':self.clear()
        self.status(state,action_status=result.status)

    def cancel(self,_msg):
        if self.nav_goal_handle is not None:
            self.status('cancelling');self.nav_goal_handle.cancel_goal_async()
        elif self.pending or self.sending:
            self.generation+=1;self.pending=False
            if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
            self.goal_handle=None;self.clear();self.status('cancelled',reason='before_execution')
        else:self.status('idle')

    def check_timeout(self):
        super().check_timeout()
        if self.sending and time.monotonic()>self.send_deadline:
            self.generation+=1;self.sending=False;self.clear()
            self.status('rejected',reason='navigation_request_wall_timeout')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a,rosargs=p.parse_known_args()
    if not a.execute:p.error('--execute required: this node sends simulated navigation goals')
    rclpy.init(args=rosargs);node=NavigationGoal()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.nav_goal_handle is not None and rclpy.ok():
            future=node.nav_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
