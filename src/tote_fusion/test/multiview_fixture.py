"""Independent analytic RGB-D and robot kinematics for sensor replay only."""
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

NAMES=['head','left','right']
FRAMES=['head_front_camera_depth_optical_frame','wrist_left_camera_depth_optical_frame','wrist_right_camera_depth_optical_frame']
YAW=np.deg2rad(25)
TRUTH_R=np.array([[np.cos(YAW),-np.sin(YAW),0],[np.sin(YAW),np.cos(YAW),0],[0,0,1]])
TRUTH_T=np.array([1.02,0,.5238])


def camera_pose(camera, slide=0, frontal=False):
    origin=np.array([[-.9,-.9,1.5],[-1.,-.8,1.4],[-.8,-1.,1.45]][camera])
    if frontal: origin=np.array([[-.9,0.,.65],[0.,-.9,.65],[-1.,0.,.68]][camera])
    forward=(np.array([0,0,.08])-origin); forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0,0,1]); right/=np.linalg.norm(right)
    rotation=TRUTH_R @ np.column_stack((right,np.cross(forward,right),forward))
    position=TRUTH_R @ origin+TRUTH_T
    if camera: position[camera-1]+=slide
    return rotation,position


def robot_description(frontal=False):
    text='<robot name="m2_tf_fixture"><link name="base_link"/>'
    for camera,frame in enumerate(FRAMES):
        r,t=camera_pose(camera,frontal=frontal)
        rpy=[np.arctan2(r[2,1],r[2,2]),np.arctan2(-r[2,0],np.hypot(r[2,1],r[2,2])),np.arctan2(r[1,0],r[0,0])]
        mount=frame if camera==0 else NAMES[camera]+'_mount'
        text+=f'<link name="{mount}"/><joint name="{NAMES[camera]}_slide" type="'+('prismatic' if camera else 'fixed')+'">'
        text+=f'<parent link="base_link"/><child link="{mount}"/><origin xyz="'+ ' '.join(map(str,t))+'" rpy="'+' '.join(map(str,rpy))+'"/>'
        # Axes are expressed in the child frame; choose body X/Y translations.
        if camera:
            axis=r.T @ np.eye(3)[camera-1]
            text+='<axis xyz="'+' '.join(map(str,axis))+'"/><limit lower="-0.4" upper="0.4" effort="1" velocity="1"/>'
        text+='</joint>'
        if camera:
            text+=f'<link name="{frame}"/><joint name="{NAMES[camera]}_optical" type="fixed"><parent link="{mount}"/><child link="{frame}"/></joint>'
    return text+'</robot>'


def sensor_frame(camera,stamp,slide,floor_only=False,dimensions=(.6,.4,.23),frontal=False):
    # Independent bounded analytic planes, not estimator-generated correspondences.
    w,h,fx=160,120,180.+camera*5
    if frontal: w,h,fx=640,480,550.+camera*5
    cx,cy=(w-1)/2,(h-1)/2
    length,width,height=dimensions
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    rays=np.stack(((u-cx)/fx,(v-cy)/fx,np.ones_like(u)),axis=-1)
    r,t=camera_pose(camera,slide,frontal); origin=TRUTH_R.T @ (t-TRUTH_T)
    directions=rays @ r.T @ TRUTH_R
    distances=[]
    bound_tolerance=1e-8 if frontal else .001
    planes=[(2,.018),(0,-length/2),(1,-width/2)]
    if frontal: planes += [(0,length/2-.0025),(1,width/2-.0025)]
    for axis,offset in planes:
        with np.errstate(divide='ignore',invalid='ignore'):
            distance=(offset-origin[axis])/directions[:,:,axis]
        points=origin+directions*distance[:,:,None]
        valid=(distance>0)&(np.abs(points[:,:,0])<=length/2+bound_tolerance)&(np.abs(points[:,:,1])<=width/2+bound_tolerance)&(points[:,:,2]>=.018-1e-8)&(points[:,:,2]<=height)
        distances.append(np.where(valid,distance,np.inf))
    surface=np.argmin(distances,axis=0); z=np.min(distances,axis=0)
    finite=np.isfinite(z)
    points=origin+directions*np.where(finite,z,0)[:,:,None]
    selected=finite & (points[:,:,2]<.173) & ((surface==0)|(points[:,:,2]>.027))
    selected &= (surface==0)|((points[:,:,2]<.079)|(points[:,:,2]>.118))
    if floor_only: selected &= (surface==0)&(np.abs(points[:,:,0])<.10)&(np.abs(points[:,:,1])<.06)
    # Geometry replay preserves actual object/background depth outside its
    # test mask, allowing free-space checks to distinguish a mask crop.
    depth_values=np.where(finite,z,2.9) if frontal else np.where(selected,z,0)
    depth=Image(width=w,height=h,encoding='32FC1',step=w*4,data=depth_values.astype('<f4').tobytes())
    depth.header.frame_id=FRAMES[camera]; depth.header.stamp=Time(nanoseconds=stamp).to_msg()
    mask=Image(width=w,height=h,encoding='mono8',step=w,data=(selected.astype('uint8')*255).tobytes());mask.header=depth.header
    info=CameraInfo(width=w,height=h,k=[fx,0.,cx,0.,fx,cy,0.,0.,1.],r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],p=[fx,0.,cx,0.,0.,fx,cy,0.,0.,0.,1.,0.]);info.header=depth.header
    return depth,mask,info


