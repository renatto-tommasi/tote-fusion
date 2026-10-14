#!/usr/bin/env python3
"""RViz documentation viewer for exported recordings; no estimator/control link."""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray


def wire(marker_id,pose,dimensions,color):
    m=Marker(id=marker_id,ns='documentation',type=Marker.LINE_LIST,action=Marker.ADD)
    m.header.frame_id='base_link';m.pose.position.x,m.pose.position.y,m.pose.position.z=[pose['position'][axis] for axis in ['x','y','z']]
    q=pose['orientation'];m.pose.orientation.x=q['x'];m.pose.orientation.y=q['y'];m.pose.orientation.z=q['z'];m.pose.orientation.w=q['w']
    m.scale.x=.0014;m.color.r,m.color.g,m.color.b=color;m.color.a=1.
    corners=np.array(list(itertools.product([-dimensions[0]/2,dimensions[0]/2],[-dimensions[1]/2,dimensions[1]/2],[0,dimensions[2]])))
    for i,j in itertools.combinations(range(8),2):
        if np.count_nonzero(corners[i]!=corners[j])==1:
            for p in [corners[i],corners[j]]:m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
    return m


def main():
    p=argparse.ArgumentParser();p.add_argument('--clouds',type=Path,required=True);p.add_argument('--model',type=Path,required=True)
    p.add_argument('--case',choices=['accepted','rejected'],required=True);p.add_argument('--stage',choices=['before','after'],required=True)
    args=p.parse_args();import yaml
    nominal=yaml.safe_load(args.model.read_text());meta=json.loads((args.clouds/'provenance.json').read_text())['cases'][args.case]
    clouds=np.load(args.clouds/(args.case+'.npz'));rclpy.init();node=rclpy.create_node('recorded_point_cloud_documentation')
    pubs=[node.create_publisher(PointCloud2,'/documentation/'+name,10) for name in ['head','left','right']]
    markers=node.create_publisher(MarkerArray,'/documentation/model',10)
    pose={'position':dict(zip(['x','y','z'],nominal['prior_pose']['position_m'])),
        'orientation':dict(zip(['x','y','z','w'],nominal['prior_pose']['orientation_xyzw']))}
    geometry=yaml.safe_load((args.model.parent/nominal['geometry_file']).read_text())['nominal_outer_dimensions_m']
    dimensions=[geometry[key] for key in ['length','width','height']];color=(1.,.56,.12)
    if args.case=='accepted' and args.stage=='after':
        candidate=meta['result']['candidate'];pose=candidate['pose']['pose'];d=candidate['proposed_outer_dimensions_m'];dimensions=[d['x'],d['y'],d['z']];color=(.2,1.,.45)
    message=MarkerArray(markers=[wire(1,pose,dimensions,color),
        Marker(id=2,ns='documentation',action=Marker.DELETE),Marker(id=3,ns='documentation',action=Marker.DELETE)])
    def publish():
        for camera,pub in zip(['head','left','right'],pubs):
            xyz=clouds[camera+('_first' if args.stage=='before' else '_all')].astype('<f4')
            pub.publish(PointCloud2(header=Header(frame_id='base_link',stamp=node.get_clock().now().to_msg()),height=1,width=len(xyz),
                fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])],
                is_bigendian=False,point_step=12,row_step=12*len(xyz),data=xyz.tobytes(),is_dense=True))
        markers.publish(message)
    node.create_timer(.2,publish)
    try:rclpy.spin(node)
    finally:node.destroy_node()


if __name__=='__main__':main()
