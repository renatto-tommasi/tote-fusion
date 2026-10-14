"""ROS action/fault-policy tests. Fixtures contain no simulator truth inputs."""
import os
from pathlib import Path
import struct
import subprocess
import time
import unittest
import sys
sys.path.insert(0,str(Path(__file__).parent))
from ros_test import RosTest

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tote_fusion_interfaces.action import EstimateTote
from tote_fusion_interfaces.msg import TotePoseCandidate


@pytest.mark.launch_test
def generate_test_description():
    domain = str(100 + os.getpid() % 100)
    os.environ['ROS_DOMAIN_ID'] = domain
    node = Node(package='tote_fusion', executable='tote_fusion_node', output='screen',
                additional_env={'ROS_DOMAIN_ID': domain}, parameters=[{
                    'use_sim_time': True,
                    'model_file': str(Path(get_package_share_directory('tote_fusion')) / 'config/tote_model.yaml'),
                    'validation_profile': 'head_sim_pi_equivalent',
                    'depth_topic': '/m1_test/depth', 'mask_topic': '/m1_test/mask',
                    'camera_info_topic': '/m1_test/info',
                    'max_frames': 2, 'wall_timeout_s': 1.0, 'pixel_stride': 1,
                }])
    return LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'process': node}


class ActionContract(RosTest):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m1_contract_test')
        cls.client = ActionClient(cls.node, EstimateTote, '/tote_fusion/estimate')
        cls.parameters = AsyncParameterClient(cls.node, '/tote_fusion')
        cls.clock_pub = cls.node.create_publisher(Clock, '/clock', 10)
        cls.depth_pub = cls.node.create_publisher(Image, '/m1_test/depth', 10)
        cls.mask_pub = cls.node.create_publisher(Image, '/m1_test/mask', 10)
        cls.info_pub = cls.node.create_publisher(CameraInfo, '/m1_test/info', 10)
        cls.clock_ns = 1_000_000_000
        cls.codes, cls.candidates = [], []
        cls.diag_sub = cls.node.create_subscription(DiagnosticArray, '/diagnostics',
            lambda m: cls.codes.extend(dict((v.key, v.value) for v in s.values)['code'] for s in m.status if s.name == 'tote_fusion'), 10)
        cls.candidate_sub = cls.node.create_subscription(TotePoseCandidate, '/tote_fusion/candidate', cls.candidates.append, 10)
        cls.heartbeat = cls.node.create_timer(.02, lambda: cls.clock_pub.publish(Clock(clock=Time(nanoseconds=cls.clock_ns).to_msg())))
        assert cls.client.wait_for_server(timeout_sec=5), 'Action server unavailable'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        type(self).clock_ns += 1_000_000_000
        self.codes.clear(); self.candidates.clear()
        self.spin_for(.15)

    def goal(self, **values):
        message = EstimateTote.Goal(object_id='m1-contract', expected_model_revision=0,
                                   collection_duration=Duration(seconds=.5).to_msg(), min_distinct_views=1,
                                   mode=EstimateTote.Goal.POSE_ONLY, validation_profile='head_sim_pi_equivalent')
        for key, value in values.items():
            setattr(message, key, value)
        return self.future(self.client.send_goal_async(message))

    def images(self):
        type(self).clock_ns += 100_000_000
        self.spin_for(.08)
        depth = Image(height=4, width=4, encoding='32FC1', step=16,
                      data=struct.pack('<16f', *([1.] * 16)))
        depth.header.frame_id = 'head_front_camera_depth_optical_frame'
        depth.header.stamp = Time(nanoseconds=self.clock_ns).to_msg()
        mask = Image(height=4, width=4, encoding='mono8', step=4, data=[255]*16)
        mask.header = depth.header
        info = CameraInfo(height=4, width=4, k=[2.,0.,1.,0.,2.,1.,0.,0.,1.],
                          r=[1.,0.,0.,0.,1.,0.,0.,0.,1.], p=[2.,0.,1.,0.,0.,2.,1.,0.,0.,0.,1.,0.])
        info.header = depth.header
        return depth, mask, info

    def publish(self, images):
        depth, mask, info = images
        self.info_pub.publish(info); self.mask_pub.publish(mask); self.depth_pub.publish(depth)

    def failure(self, handle, code, status=GoalStatus.STATUS_ABORTED):
        reply = self.future(handle.get_result_async())
        assert reply.status == status and reply.result.code == code and not reply.result.success
        self.wait(lambda: code in self.codes)
        assert not self.candidates and not reply.result.candidate.candidate_id

    def test_stale_model_and_unsupported_mode_are_rejected(self):
        for values, code in [({'expected_model_revision': 1}, 'STALE_MODEL_REVISION'),
                             ({'mode': EstimateTote.Goal.POSE_AND_GEOMETRY}, 'UNSUPPORTED_MODE')]:
            assert not self.goal(**values).accepted
            self.wait(lambda: code in self.codes)
        assert not self.candidates

    def test_concurrent_request_is_rejected_and_cancel_releases_session(self):
        handle = self.goal()
        assert handle.accepted and not self.goal().accepted
        self.wait(lambda: 'BUSY' in self.codes)
        response = self.future(handle.cancel_goal_async())
        assert response.goals_canceling
        self.failure(handle, 'CANCELED', GoalStatus.STATUS_CANCELED)
        following = self.goal()
        assert following.accepted
        self.future(following.cancel_goal_async())
        self.failure(following, 'CANCELED', GoalStatus.STATUS_CANCELED)

    def test_invalid_depth_fails_without_a_candidate(self):
        handle = self.goal(); images = self.images(); images[0].encoding = 'bad'
        self.publish(images); self.failure(handle, 'INVALID_DEPTH')

    def test_mask_dimensions_must_match(self):
        handle = self.goal(); images = self.images(); images[1].width = 3
        self.publish(images); self.failure(handle, 'MASK_MISMATCH')

    def test_missing_capture_time_tf_fails(self):
        handle = self.goal(); images = self.images()
        images[2].p[3] = -0.03  # Valid stereo metadata; axial-depth projection uses K in its optical frame.
        self.publish(images)
        self.failure(handle, 'TF_UNAVAILABLE')

    def test_stale_frame_is_rejected(self):
        handle = self.goal(); images = self.images()
        images[0].header.stamp = Time(nanoseconds=self.clock_ns-1_000_000_000).to_msg()
        self.depth_pub.publish(images[0]); self.failure(handle, 'STALE_OBSERVATION')

    def test_fresh_pretrigger_frame_is_excluded_and_cannot_supply_a_candidate(self):
        handle = self.goal()
        depth, _, _ = self.images()
        depth.header.stamp = Time(nanoseconds=self.clock_ns-200_000_000).to_msg()
        self.depth_pub.publish(depth)
        self.wait(lambda: 'OUTSIDE_COLLECTION' in self.codes)
        type(self).clock_ns += 600_000_000
        self.failure(handle, 'INSUFFICIENT_DATA')

    def test_clock_reset_invalidates_collection(self):
        handle = self.goal(); type(self).clock_ns = 0
        self.failure(handle, 'SIM_TIME_RESET')

    def test_paused_clock_does_not_disable_wall_timeout(self):
        handle = self.goal()
        self.failure(handle, 'TIMEOUT')

    def test_completed_empty_collection_rejects(self):
        handle = self.goal(); type(self).clock_ns += 600_000_000
        self.failure(handle, 'INSUFFICIENT_DATA')

    def test_unmatched_image_queue_has_a_hard_bound(self):
        handle = self.goal()
        for _ in range(3):
            depth, _, _ = self.images()
            self.depth_pub.publish(depth)
        self.failure(handle, 'BUFFER_LIMIT')

    def test_model_configuration_cannot_change(self):
        assert self.parameters.wait_for_services(timeout_sec=2)
        reply = self.future(self.parameters.set_parameters([Parameter('model_file', value='/invalid/new-model.yaml')]))
        assert not reply.results[0].successful
        values = self.future(self.parameters.get_parameters(['model_file']))
        assert values.values[0].string_value.endswith('/config/tote_model.yaml')

    def test_invalid_startup_parameters_fail_instead_of_using_defaults(self):
        model = str(Path(get_package_share_directory('tote_fusion')) / 'config/tote_model.yaml')
        for argument, error in [('max_observation_age_s:=0.0000000001', 'Invalid duration'),
                                ('misspelled_capture_limit:=1', 'Unknown parameter')]:
            reply = subprocess.run(['ros2','run','tote_fusion','tote_fusion_node','--ros-args',
                '-r','__node:=m1_invalid_configuration','-p','use_sim_time:=true',
                '-p','model_file:='+model,'-p',argument],capture_output=True,text=True,timeout=5)
            assert reply.returncode != 0 and error in reply.stderr


@launch_testing.post_shutdown_test()
class ProcessExit(unittest.TestCase):
    def test_exit(self, proc_info, process):
        launch_testing.asserts.assertExitCodes(proc_info, process=process)
