"""A slow native planning reply must not make cached receipts authoritative."""
from concurrent.futures import Future
from pathlib import Path
import sys
import struct
import tempfile
import threading
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from shapely.geometry import box

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_rolling_window_manager import RollingWindowManager,cached_grid_wire_message
from osmag_rolling_raster import RollingRaster
from osmag_constraint_mask import build_mask
from osmag_mission_contract import MissionError
from osmag_window_admission import WindowAdmission


def pose_message():
    return NS(header=NS(frame_id=''),pose=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.)))


class PreflightFreshnessTests(unittest.TestCase):
    def test_window_tick_defers_stale_pose_and_propagates_other_failures(self):
        with tempfile.TemporaryDirectory() as folder:
            manager = self.manager(Path(folder), True)
            manager.frozen = False
            manager.committed = manager.pending
            manager.pending = None
            manager.adapter.enabled = True
            manager.adapter.model = NS(state='executing')

            def stale_pose():
                raise MissionError('stale_robot_tf')

            manager.last_tick_wall = float('-inf')
            manager.adapter.pose = stale_pose
            manager.tick()
            self.assertIsNone(manager.pending)
            self.assertFalse(manager.events)

            def invalid_pose():
                raise MissionError('invalid_tf_clock')

            manager.last_tick_wall = float('-inf')
            manager.adapter.pose = invalid_pose
            with self.assertRaisesRegex(MissionError, 'invalid_tf_clock'):
                manager.tick()

    def test_heartbeat_patch_preserves_occupancy_payload_and_checks_wire_layout(self):
        stamp = NS(sec=1,nanosec=2)
        msg = NS(header=NS(stamp=stamp))
        def serialize(message):
            time = message.header.stamp
            return b'\x00\x01\x00\x00'+struct.pack('<iI',time.sec,time.nanosec)+b'grid-data'

        wire = cached_grid_wire_message(msg, serialize)
        self.assertIs(msg.header.stamp, stamp)
        new_stamp = NS(sec=3,nanosec=5)
        self.assertEqual(wire(new_stamp), serialize(NS(header=NS(stamp=new_stamp))))

        # If the serializer changes any field outside the stamp slot, fall
        # back to typed publish even when the first timestamp matches.
        def different_layout(message):
            return serialize(message) + struct.pack('<i',message.header.stamp.sec)
        self.assertIsNone(cached_grid_wire_message(msg,different_layout))

    def test_cached_grid_still_expires_and_replacement_is_checked(self):
        with tempfile.TemporaryDirectory() as folder:
            manager = self.manager(Path(folder), True)
            state = manager.pending
            gate = state['gate']
            self.assertTrue(manager.observe(state))
            self.assertTrue(manager.observe(state))
            self.assertEqual(gate.observations['global']['stamp'], 1.)

            manager.clock = 1.51
            gate.note_source_publication(manager.clock, state['window'].sha256)
            self.assertFalse(manager.observe(state))
            self.assertEqual(gate.observations, {})

            original = manager.grids['global']
            manager.grids['global'] = NS(
                **{**vars(original), 'header': NS(frame_id='odom', stamp=NS(sec=1,nanosec=510000000))})
            self.assertFalse(manager.observe(state))
            self.assertEqual(state['last_errors']['global'], 'costmap_transform_required')

    def test_durable_mask_publishes_once_per_revision_with_live_grid_heartbeat(self):
        with tempfile.TemporaryDirectory() as folder:
            manager = self.manager(Path(folder), True)
            manager.mask_on_revision = True
            manager.frozen = manager.closed = False
            manager.publications = []
            manager.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: manager.clock)))
            grids, masks = [], []
            manager.pub = NS(publish=lambda msg: grids.append(msg.header.stamp))
            manager.mask_pub = NS(publish=lambda msg: masks.append(msg.header.stamp))
            manager.pending.update(heartbeat_messages=(NS(header=NS()), NS(header=NS())),
                                   heartbeat_wire_messages=(None, None))
            manager.heartbeat()
            manager.clock = 1.1
            manager.heartbeat()
            self.assertEqual(grids, [1., 1.1])
            self.assertEqual(masks, [1.])
            self.assertEqual(manager.pending['gate'].last_source_stamp, 1.1)
            manager.pending = {k: v for k, v in manager.pending.items() if k != 'mask_published'}
            manager.clock = 1.2
            manager.heartbeat()
            self.assertEqual(grids, [1., 1.1, 1.2])
            self.assertEqual(masks, [1., 1.2])

    def manager(self,folder,fresh_updates):
        leaves={'a':box(-3,-3,0,3),'b':box(0,-3,3,3)}
        passages={'p':[(0.,-1.),(0.,1.)]}
        window=RollingRaster(leaves,passages,resolution=.1,anchor=(-5.,-5.),shape=(80,80)).render(
            (0.,0.),revision=1,stamp=1.)
        snapshot=build_mask(window.grid,window.resolution,window.origin,leaves,passages,{'p':('a','b')},
            allowed_areas=['a','b'],open_passages=['p'],disabled_passages=[],robot_pose=(-1.,0.),mission_id='test')
        gate=WindowAdmission(window,snapshot,1.,[(-1.,0.)])
        m=RollingWindowManager.__new__(RollingWindowManager)
        m.pending=dict(window=window,snapshot=snapshot,gate=gate,published_sim=1.,pose=[-1.,0.,0.],
            raster_s=0.,mask_and_overlap_s=0.,common_native_plan=[])
        m.grids={};m.gate_lock=threading.RLock();m.publish_lock=threading.RLock()
        m.output=folder;m.file_prefix='';m.records=[];m.events=[];m.committed=None;m.busy=False
        m.clock=1.;m.requests=0
        m.adapter=NS(sim=lambda:m.clock,pose=lambda:([-1.,0.,0.],m.clock),scan_stamp=10.)
        m.contract=NS(path=[[-1.,0.],[1.,0.]])
        m.preflight_segments=lambda:[NS(index=0,point=(.2,0.),yaw=0.),NS(index=1,point=(1.,0.),yaw=0.)]
        def publish():
            for key in ['global','local']:
                m.grids[key]=NS(header=NS(frame_id='map',stamp=NS(sec=int(m.clock),nanosec=round((m.clock%1)*1e9))),
                    info=NS(height=80,width=80,resolution=window.resolution,
                        origin=NS(position=NS(x=window.origin[0],y=window.origin[1]))),data=snapshot.combined.ravel().tolist())
        publish()
        def wait(predicate,seconds=30):
            if not predicate():raise TimeoutError('fresh received costmaps unavailable')
        m.wait=wait
        def send(request):
            m.requests+=1
            def reply():
                # A native action completes after the old .5s/.75s receipt
                # expires. Fresh messages may have arrived while it was pending.
                m.clock+=.8
                gate.note_source_publication(m.clock,window.sha256)
                if fresh_updates:publish()
                points=[pose_message(),pose_message()]
                points[0].pose.position.x=-1.
                points[1].pose.position.x=request.goal.pose.position.x
                done=Future();done.set_result(NS(status=4,result=NS(path=NS(poses=points))))
                return done
            future=Future();future.set_result(NS(accepted=True,get_result_async=reply))
            return future
        m.client=NS(server_is_ready=lambda:True,send_goal_async=send)
        return m

    def modules(self):
        action=ModuleType('nav2_msgs.action');action.ComputePathToPose=NS(Goal=lambda:NS())
        geometry=ModuleType('geometry_msgs.msg');geometry.PoseStamped=pose_message
        return {'nav2_msgs':ModuleType('nav2_msgs'),'nav2_msgs.action':action,
                'geometry_msgs':ModuleType('geometry_msgs'),'geometry_msgs.msg':geometry}

    def test_fresh_costmaps_received_during_slow_query_are_revalidated(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(sys.modules,self.modules()):
            m=self.manager(Path(folder),True)
            m.prepare()
            self.assertIsNotNone(m.committed)
            self.assertEqual(m.requests,2)
            self.assertIsNone(m.pending)
            self.assertEqual(len(m.records),1)
            for value in m.records[0]['preflights'].values():
                self.assertLessEqual(value['admission']['observed_at']-value['admission']['source_last_publication'],.5)

    def test_source_heartbeat_cannot_freshen_missing_native_updates(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(sys.modules,self.modules()):
            m=self.manager(Path(folder),False)
            with self.assertRaisesRegex(TimeoutError,'fresh received costmaps unavailable'):m.prepare()
            self.assertIsNone(m.committed)
            self.assertEqual(m.requests,1)
            self.assertFalse(m.records)
            self.assertFalse(m.busy)


if __name__=='__main__':unittest.main()
