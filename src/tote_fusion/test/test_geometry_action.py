"""Geometry ROS oracle: independent analytic RGB-D, never a runtime truth topic."""
import os
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent))
from test_multiview import MultiviewFixture
from multiview_fixture import NAMES,FRAMES,TRUTH_T,YAW,sensor_frame,camera_pose,robot_description
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
from tote_fusion_interfaces.action import EstimateTote


@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID']='88'
    estimator=Node(package='tote_fusion',executable='tote_fusion_node',output='screen',parameters=[{
        'use_sim_time':True,'model_file':str(Path(get_package_share_directory('tote_fusion'))/'config/tote_model.yaml'),
        'camera_ids':NAMES,'validation_profile':'geometry_sim_pi_equivalent','reliable_sensor_qos':True,
        'pixel_stride':4,'wall_timeout_s':5.,'candidate_validity_s':2.,'max_frames':18,
        'depth_topic':'/m2/head/depth','mask_topic':'/m2/head/mask','camera_info_topic':'/m2/head/info',
        **{f'{name}.{key}_topic':f'/m2/{name}/{suffix}' for name in NAMES[1:] for key,suffix in [('depth','depth'),('mask','mask'),('camera_info','info')]}}])
    robot=Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[{
        'robot_description':robot_description(frontal=True),'use_sim_time':True,'publish_frequency':100.}])
    return LaunchDescription([estimator,robot,launch_testing.actions.ReadyToTest()]),{'process':estimator,'robot':robot}


class GeometryAction(MultiviewFixture):
    profile='geometry_sim_pi_equivalent'
    mode=EstimateTote.Goal.POSE_AND_GEOMETRY
    camera_pose=staticmethod(lambda camera,slide=0:camera_pose(camera,slide,frontal=True))
    dimensions=(.62,.42,.23)
    scenario='clean'

    def render(self,camera,stamp,slide,floor_only=False):
        dimensions=self.dimensions
        if self.scenario=='conflict' and camera==2:dimensions=(.62,.405,.23)
        depth,mask,info=sensor_frame(camera,stamp,slide,floor_only,dimensions=dimensions,frontal=True)
        if self.scenario=='crop':
            selected=np.frombuffer(bytes(mask.data),dtype=np.uint8).reshape(mask.height,mask.width).copy()
            selected[:,:mask.width//3]=0;selected[:,2*mask.width//3:]=0
            mask.data=selected.tobytes()
        return depth,mask,info

    def setUp(self):
        self.dimensions=(.62,.42,.23);self.scenario='clean';super().setUp()

    def verify(self,candidate,names):
        pose=candidate.pose.pose
        assert np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-TRUTH_T)<.004
        angle=2*np.arccos(min(1.,abs(pose.orientation.z*np.sin(YAW/2)+pose.orientation.w*np.cos(YAW/2))))
        assert angle<np.deg2rad(.5)
        assert candidate.estimation_mode==1 and candidate.parameter_names==names
        values=dict(zip(candidate.parameter_names,candidate.parameter_values_m))
        for name,value in values.items():assert abs(value-{'width':.42,'length':.62}[name])<.004,(name,value)
        n=6+len(names);cov=np.array(candidate.joint_covariance).reshape(n,n)
        assert np.allclose(cov,cov.T,atol=1e-12) and np.linalg.eigvalsh(cov).min()>0
        assert np.linalg.norm(cov[:6,6:])>1e-9 and candidate.observable_rank==n
        assert candidate.fixed_geometry_boundary_rmse_m>.007 and candidate.held_out_boundary_rmse_m<.005
        assert candidate.parent_model_revision==0 and len(candidate.proposed_geometry_hash)==64
        assert candidate.proposed_geometry_hash!=candidate.geometry_hash and not candidate.covariance_calibrated
        assert 'height' in candidate.unresolved_geometry and 'wall_thickness' in candidate.unresolved_geometry
        assert candidate.geometry_generator=='EU6423_17_boxes_v1'
        for axis,name,nominal in [('x','length',.6),('y','width',.4)]:
            extent=getattr(candidate.collision_upper_m,axis)-getattr(candidate.collision_lower_m,axis)
            assert extent>=nominal+.004
            if name in values:assert extent>=values[name]+.004
        assert candidate.collision_lower_m.z==-.002 and candidate.collision_upper_m.z==.232
        assert candidate.boundary_support_counts and min(candidate.boundary_support_counts)>=24

    def test_width_then_length_joint_candidate_and_conservative_envelope(self):
        reply=self.collect(check_accuracy=False);assert reply.success,(reply.code,reply.message)
        self.verify(reply.candidate,['width','length'])
        assert not reply.candidate.fixed_geometry_converged and reply.candidate.fixed_geometry_iterations==40
        assert reply.candidate.solver_iterations>80

    def test_only_width_supported_leaves_length_unresolved(self):
        self.dimensions=(.6,.42,.23)
        reply=self.collect(active=(0,2),minimum=2,check_accuracy=False);assert reply.success,(reply.code,reply.message)
        self.verify(reply.candidate,['width']);assert 'length' in reply.candidate.unresolved_geometry
        assert reply.candidate.proposed_outer_dimensions_m.x==.6

    def test_moving_camera_geometry_uses_capture_time_interpolation(self):
        reply=self.collect(moving=True,interpolate=True,check_accuracy=False);assert reply.success,(reply.code,reply.message)
        self.verify(reply.candidate,['width','length'])

    def test_mask_crop_cannot_become_a_geometry_candidate(self):
        self.scenario='crop'
        reply=self.collect(check_accuracy=False);assert reply.code=='INSUFFICIENT_GEOMETRY',(reply.code,reply.message)

    def test_conflicting_camera_widths_reject_without_publication(self):
        self.scenario='conflict'
        reply=self.collect(check_accuracy=False);assert reply.code=='GEOMETRY_CONFLICT',(reply.code,reply.message)

    def test_geometry_cancel_and_reset_keep_candidate_topic_empty(self):
        for control,code in [('cancel','CANCELED'),('brief_reset','SIM_TIME_RESET')]:
            self.setUp();reply=self.collect(control=control,check_accuracy=False);assert reply.code==code,(reply.code,reply.message)


@launch_testing.post_shutdown_test()
class Shutdown(unittest.TestCase):
    def test_exit(self,proc_info,process,robot):
        launch_testing.asserts.assertExitCodes(proc_info,process=process)
        launch_testing.asserts.assertExitCodes(proc_info,process=robot)
