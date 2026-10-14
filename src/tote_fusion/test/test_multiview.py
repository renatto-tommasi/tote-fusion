"""Acquisition-time joint/TF fixtures, separate from the Gazebo integration."""
import json
import os
from pathlib import Path
import sys
import time
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from ros_test import RosTest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from tote_fusion_interfaces.action import EstimateTote
from tote_fusion_interfaces.msg import TotePoseCandidate

from multiview_fixture import NAMES, FRAMES, TRUTH_T, YAW, camera_pose, robot_description, sensor_frame


@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='87'
    model=str(Path(get_package_share_directory('tote_fusion'))/'config/tote_model.yaml')
    estimator=Node(package='tote_fusion',executable='tote_fusion_node',output='screen',parameters=[{
        'use_sim_time':True,'model_file':model,'camera_ids':NAMES,'validation_profile':'multi_sim_pi_equivalent',
        'reliable_sensor_qos':True,'pixel_stride':2,'wall_timeout_s':5.,'candidate_validity_s':2.,'max_frames':18,
        'depth_topic':'/m2/head/depth','mask_topic':'/m2/head/mask','camera_info_topic':'/m2/head/info',
        **{f'{name}.{key}_topic':f'/m2/{name}/{suffix}' for name in NAMES[1:] for key,suffix in [('depth','depth'),('mask','mask'),('camera_info','info')]}}])
    robot=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[{
        'robot_description':robot_description(),'use_sim_time':True,'publish_frequency':100.}])
    return LaunchDescription([estimator,robot,launch_testing.actions.ReadyToTest()]),{'process':estimator,'robot':robot}


class MultiviewFixture(RosTest):
    profile='multi_sim_pi_equivalent'
    mode=0
    render=staticmethod(sensor_frame)
    camera_pose=staticmethod(camera_pose)
    @classmethod
    def setUpClass(cls):
        rclpy.init();cls.node=rclpy.create_node('independent_m2_tf_fixture')
        cls.client=ActionClient(cls.node,EstimateTote,'/tote_fusion/estimate')
        cls.clock_pub=cls.node.create_publisher(Clock,'/clock',10)
        cls.joints=cls.node.create_publisher(JointState,'/joint_states',10)
        cls.raw_tf=cls.node.create_publisher(TFMessage,'/tf',10)
        cls.raw_static=cls.node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.pubs=[{topic:cls.node.create_publisher(kind,f'/m2/{name}/{topic}',10) for topic,kind in [('depth',Image),('mask',Image),('info',CameraInfo)]} for name in NAMES]
        cls.candidates=[];cls.sub=cls.node.create_subscription(TotePoseCandidate,'/tote_fusion/candidate',cls.candidates.append,10)
        cls.buffer=Buffer();cls.listener=TransformListener(cls.buffer,cls.node)
        cls.clock_ns=1_000_000_000
        cls.heartbeat=cls.node.create_timer(.02,lambda:cls.clock_pub.publish(Clock(clock=Time(nanoseconds=cls.clock_ns).to_msg())))
        assert cls.client.wait_for_server(timeout_sec=8)

    @classmethod
    def tearDownClass(cls): cls.node.destroy_node();rclpy.shutdown()

    def setUp(self):
        type(self).clock_ns+=3_000_000_000;self.candidates.clear();self.spin_for(.15)
        self.wait(lambda:self.buffer.can_transform('base_link',FRAMES[0],Time()))

    def collect(self,active=(0,1,2),minimum=3,moving=False,floor_only=False,control=None,interpolate=False,check_accuracy=True,unrelated_camera=None):
        self.candidates.clear()
        self.clock_pub.publish(Clock(clock=Time(nanoseconds=self.clock_ns).to_msg()))
        self.spin_for(.10)  # Synchronize the fixture clock before accepting another goal.
        start=self.clock_ns
        goal=EstimateTote.Goal(object_id='m2-fixture',expected_model_revision=0,min_distinct_views=minimum,
            collection_duration=Duration(seconds=1.2).to_msg(),mode=self.mode,validation_profile=self.profile)
        self.goal_wall_started=time.monotonic()
        handle=self.future(self.client.send_goal_async(goal));assert handle.accepted
        result=handle.get_result_async(); captures=[]
        for frame in range(6):
            stamp=start+50_000_000+frame*100_000_000
            older=[.015*frame,-.012*frame] if moving else [0.,0.]
            newer=[q+.08 for q in older] if moving else older
            
            before=stamp-40_000_000 if interpolate else stamp
            positions_before=[q-.08 for q in older] if interpolate else older
            for joint_stamp,positions in [(before,positions_before),(stamp+40_000_000,newer)]:
                type(self).clock_ns=joint_stamp;self.spin_for(.03)
                self.joints.publish(JointState(header=rclpy_header(joint_stamp),name=['left_slide','right_slide'],position=positions))
                self.spin_for(.03)
            for camera in active:
                if camera:
                    self.wait(lambda c=camera:self.buffer.can_transform('base_link',FRAMES[c],Time(nanoseconds=stamp+40_000_000)))
                messages=self.render(camera,stamp,older[camera-1] if camera else 0,floor_only)
                if camera==unrelated_camera:
                    depth=messages[0]
                    values=np.frombuffer(bytes(depth.data),dtype='<f4')
                    depth.data=np.where(values>0,2.8,0).astype('<f4').tobytes()
                for topic,message in zip(('info','mask','depth'),(messages[2],messages[1],messages[0])):self.pubs[camera][topic].publish(message)
                captures.append((NAMES[camera],stamp))
            self.spin_for(.04)
        if moving:
            for camera in active:
                if camera:
                    old=self.buffer.lookup_transform('base_link',FRAMES[camera],Time(nanoseconds=stamp)).transform.translation
                    latest=self.buffer.lookup_transform('base_link',FRAMES[camera],Time()).transform.translation
                    assert np.linalg.norm(np.array([old.x-latest.x,old.y-latest.y,old.z-latest.z]))>.05,(camera,stamp,old,latest)
                    assert np.linalg.norm(np.array([old.x,old.y,old.z])-self.camera_pose(camera,older[camera-1])[1])<1e-8
        if control=='brief_reset':
            self.heartbeat.cancel();zero_busy=[]
            subscription=self.node.create_subscription(DiagnosticArray,'/diagnostics',
                lambda m:zero_busy.extend(s.message for s in m.status if m.header.stamp.sec==0 and m.header.stamp.nanosec==0 and s.message.startswith('BUSY:')),10)
            try:
                self.clock_pub.publish(Clock(clock=Time(nanoseconds=0).to_msg()))
                duplicate=self.future(self.client.send_goal_async(goal));assert not duplicate.accepted
                self.spin_for(.001)
                self.clock_pub.publish(Clock(clock=Time(nanoseconds=self.clock_ns).to_msg()))
                self.brief_reset_busy_proof=zero_busy.copy()
            finally:
                self.heartbeat.reset();self.node.destroy_subscription(subscription)
        if control=='cancel':self.future(handle.cancel_goal_async())
        if control=='reset':type(self).clock_ns=0
        else:type(self).clock_ns=start+1_300_000_000
        reply=self.future(result,timeout=6).result
        self.result_wall_finished=time.monotonic()
        if reply.success:
            self.wait(lambda:len(self.candidates)==1)
            candidate=reply.candidate
            assert candidate.schema_version==3 and candidate.source_camera_ids==[NAMES[c] for c in active]
            lookup_stamps=[s.sec*10**9+s.nanosec for s in candidate.tf_lookup_stamps]
            assert sorted(zip(candidate.tf_lookup_camera_ids,lookup_stamps))==sorted(captures)
            assert candidate.sample_count==len(captures) and list(candidate.source_frame_counts)==[6]*len(active)
            assert len(set(candidate.calibration_ids))==len(active)
            pose=candidate.pose.pose
            if check_accuracy: assert np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-TRUTH_T)<.002
            angle=2*np.arccos(min(1.,abs(pose.orientation.z*np.sin(YAW/2)+pose.orientation.w*np.cos(YAW/2))))
            if check_accuracy: assert angle<np.deg2rad(.3)
        else:assert not self.candidates and not reply.candidate.candidate_id
        return reply



class MultiviewAction(MultiviewFixture):
    def test_two_wrist_cameras_with_equal_acquisition_stamps_remain_separate(self):
        assert self.collect().code=='OK'

    def test_delayed_wrist_tf_rejects_without_latest_substitution(self):
        goal=EstimateTote.Goal(object_id='delayed-tf',expected_model_revision=0,min_distinct_views=1,
            collection_duration=Duration(seconds=1.2).to_msg(),mode=0,validation_profile='multi_sim_pi_equivalent')
        handle=self.future(self.client.send_goal_async(goal));assert handle.accepted
        type(self).clock_ns+=50000000;self.spin_for(.04)
        depth,mask,info=sensor_frame(1,self.clock_ns,0)
        self.pubs[1]['info'].publish(info);self.pubs[1]['mask'].publish(mask);self.pubs[1]['depth'].publish(depth)
        result=self.future(handle.get_result_async()).result
        assert result.code=='TF_UNAVAILABLE' and not result.success and not self.candidates

    def test_moving_wrist_uses_joint_tf_at_capture_not_latest(self):
        assert self.collect(active=(0,1),minimum=2,moving=True).code=='DEGRADED_OK'

    def test_moving_wrist_interpolates_bracketing_joint_transforms(self):
        assert self.collect(moving=True,interpolate=True).code=='OK'

    def test_camera_loss_is_explicit_and_goal_controls_reduced_coverage(self):
        result=self.collect(active=(0,1),minimum=2)
        assert result.code=='DEGRADED_OK' and result.candidate.missing_camera_ids==['right']
        assert self.collect(active=(0,1),minimum=3).code=='INSUFFICIENT_DATA'

    def test_remaining_planar_camera_cannot_become_observable(self):
        assert self.collect(active=(1,),minimum=1,floor_only=True).code=='UNOBSERVABLE'

    def test_received_camera_without_model_support_cannot_count_as_a_view(self):
        assert self.collect(active=(0,1),minimum=2,unrelated_camera=1).code=='INSUFFICIENT_DATA'

    def test_pending_budget_counts_all_camera_stamp_pairs(self):
        goal=EstimateTote.Goal(object_id='overload',expected_model_revision=0,min_distinct_views=3,
            collection_duration=Duration(seconds=1.2).to_msg(),mode=0,validation_profile='multi_sim_pi_equivalent')
        handle=self.future(self.client.send_goal_async(goal));assert handle.accepted
        result=handle.get_result_async();start=self.clock_ns
        for frame in range(19):
            camera=frame%3;stamp=start+(frame//3+1)*10000000
            type(self).clock_ns=stamp;self.spin_for(.03)
            self.pubs[camera]['depth'].publish(sensor_frame(camera,stamp,0)[0]);self.spin_for(.02)
        reply=self.future(result,timeout=6).result
        assert reply.code=='BUFFER_LIMIT' and not self.candidates and not reply.candidate.candidate_id

    def test_cancel_and_reset_discard_all_camera_buffers(self):
        assert self.collect(control='cancel').code=='CANCELED'
        assert self.collect(control='reset').code=='SIM_TIME_RESET'

    def test_processed_brief_clock_reset_cannot_publish_candidate(self):
        for trial in range(5):
            if trial:self.setUp()
            reply=self.collect(active=(0,1),minimum=2,control='brief_reset')
            assert reply.code=='SIM_TIME_RESET',(trial,reply.code,self.brief_reset_busy_proof)

    def test_static_topology_and_timestamp_changes_abort_used_chains(self):
        import copy
        for fault in ('static','parent','classification','timestamp'):
            type(self).clock_ns+=2_000_000_000;self.candidates.clear();self.spin_for(.1)
            counts=[]
            goal=EstimateTote.Goal(object_id='tf-'+fault,expected_model_revision=0,min_distinct_views=1,
                collection_duration=Duration(seconds=1.2).to_msg(),mode=0,validation_profile='multi_sim_pi_equivalent')
            handle=self.future(self.client.send_goal_async(goal,feedback_callback=lambda m:counts.append(m.feedback.accepted_frames)))
            assert handle.accepted
            type(self).clock_ns+=50_000_000;stamp=self.clock_ns;self.spin_for(.03)
            self.joints.publish(JointState(header=rclpy_header(stamp),name=['left_slide','right_slide'],position=[0.,0.]))
            self.spin_for(.04)
            camera=1 if fault=='timestamp' else 0
            self.wait(lambda:self.buffer.can_transform('base_link',FRAMES[camera],Time(nanoseconds=stamp)))
            depth,mask,info=sensor_frame(camera,stamp,0)
            for name,message in [('info',info),('mask',mask),('depth',depth)]:self.pubs[camera][name].publish(message)
            self.wait(lambda:any(n>=1 for n in counts))
            child='left_mount' if fault=='timestamp' else FRAMES[0]
            original=self.buffer.lookup_transform('base_link',child,Time(nanoseconds=stamp))
            changed=copy.deepcopy(original)
            if fault in ('static','timestamp'):changed.transform.translation.x+=.004
            if fault=='parent':changed.header.frame_id='changed_mount'
            publisher=self.raw_tf if fault in ('classification','timestamp') else self.raw_static
            publisher.publish(TFMessage(transforms=[changed]))
            reply=self.future(handle.get_result_async()).result
            assert reply.code=='TF_CHANGED' and not reply.success and not self.candidates,(fault,reply.code)
            if fault!='timestamp':self.raw_static.publish(TFMessage(transforms=[original]));self.spin_for(.05)

    def test_wide_dynamic_ancestor_gap_rejects_even_when_tf2_can_interpolate(self):
        # A new mount has no stale samples from earlier trials. Static optical
        # calibration changes outside a request are explicit fresh chains.
        type(self).clock_ns+=10_000_000_000;self.spin_for(.1)
        start=self.clock_ns;stamp=start+250_000_000
        self.joints.publish(JointState(header=rclpy_header(start),name=['left_slide','right_slide'],position=[0.,0.]))
        self.wait(lambda:self.buffer.can_transform('base_link','left_mount',Time(nanoseconds=start)))
        orientation=self.buffer.lookup_transform('base_link','left_mount',Time(nanoseconds=start)).transform.rotation
        optical=TransformStamped(header=rclpy_header(0),child_frame_id=FRAMES[1])
        optical.header.frame_id='sparse_mount';optical.transform.rotation.w=1.
        self.raw_static.publish(TFMessage(transforms=[optical]));self.spin_for(.1)
        goal=EstimateTote.Goal(object_id='wide-gap',expected_model_revision=0,min_distinct_views=1,
            collection_duration=Duration(seconds=1.2).to_msg(),mode=0,validation_profile='multi_sim_pi_equivalent')
        handle=self.future(self.client.send_goal_async(goal));assert handle.accepted
        samples=[]
        for delta,slide in [(-4_000_000_000,-.08),(4_000_000_000,.08)]:
            _,position=camera_pose(1,slide)
            sample=TransformStamped(header=rclpy_header(stamp+delta),child_frame_id='sparse_mount')
            sample.header.frame_id='base_link'
            sample.transform.translation.x,sample.transform.translation.y,sample.transform.translation.z=map(float,position)
            sample.transform.rotation=orientation
            samples.append(sample)
        self.raw_tf.publish(TFMessage(transforms=samples));self.spin_for(.1)
        self.wait(lambda:self.buffer.can_transform('base_link',FRAMES[1],Time(nanoseconds=stamp)))
        midpoint=self.buffer.lookup_transform('base_link',FRAMES[1],Time(nanoseconds=stamp)).transform.translation
        assert np.linalg.norm(np.array([midpoint.x,midpoint.y,midpoint.z])-camera_pose(1)[1])<1e-8
        type(self).clock_ns=stamp;self.spin_for(.04)
        depth,mask,info=sensor_frame(1,stamp,0)
        for name,message in [('info',info),('mask',mask),('depth',depth)]:self.pubs[1][name].publish(message)
        reply=self.future(handle.get_result_async()).result
        assert reply.code=='TF_UNAVAILABLE' and not reply.success and not self.candidates
        optical.header.frame_id='left_mount'
        self.raw_static.publish(TFMessage(transforms=[optical]));self.spin_for(.1)


def rclpy_header(stamp):
    from std_msgs.msg import Header
    return Header(stamp=Time(nanoseconds=stamp).to_msg())


@launch_testing.post_shutdown_test()
class ProcessExit(unittest.TestCase):
    def test_exit(self,proc_info,process,robot):
        launch_testing.asserts.assertExitCodes(proc_info,process=process)
        launch_testing.asserts.assertExitCodes(proc_info,process=robot)
