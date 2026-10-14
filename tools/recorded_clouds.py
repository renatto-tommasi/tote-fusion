#!/usr/bin/env python3
"""Export recorded masked points for documentation; never an estimator input.

Uses saved depth, mask, CameraInfo and acquisition-time TF. No truth query.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    report=json.loads(args.report.read_text())
    cases={'accepted':93,'rejected':90}
    # Capture groups, independent of assumed wall-clock scheduling.
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    messages=[];stamps=set()
    while reader.has_next():
        topic,data,_=reader.read_next();message=deserialize_message(data,types[topic])
        messages.append((topic,message))
        if topic=='/m2/head/depth':stamps.add(message.header.stamp.sec*10**9+message.header.stamp.nanosec)
    groups=[]
    for stamp in sorted(stamps):
        if not groups or stamp-groups[-1][-1]>200_000_000:groups.append([])
        groups[-1].append(stamp)
    assert len(groups)==len(report['trials'])==125
    metadata={'source_bag_sha256':hashlib.sha256(next(args.bag.glob('*.mcap')).read_bytes()).hexdigest(),
        'source_report_sha256':hashlib.sha256(args.report.read_bytes()).hexdigest(),
        'scope':'Recorded analytic RGB-D simulation; test-only masks; visualization only.',
        'sampling':'Every fourth pixel, matching the estimator projection stride; finite masked 0.3..3 m axial depth.',
        'after_fusion':'Union of capture-time body points with candidate model overlay; points are not denoised or moved by the fit.',
        'cases':{}}
    for label,index in cases.items():
        trial=report['trials'][index];captures=groups[index]
        assert trial['scenario']=='noise' and trial['active_cameras']==3
        assert trial['result']['success']==(label=='accepted')
        buffer=Buffer(cache_time=Duration(seconds=60));frames={}
        for topic,message in messages:
            if topic in ('/tf','/tf_static'):
                for t in message.transforms:
                    stamp=t.header.stamp.sec*10**9+t.header.stamp.nanosec
                    if topic=='/tf_static':buffer.set_transform_static(t,'recorded')
                    elif captures[0]-100_000_000<=stamp<=captures[-1]+100_000_000:buffer.set_transform(t,'recorded')
            elif topic.startswith('/m2/'):
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                if stamp in captures:frames.setdefault((topic.split('/')[2],stamp),{})[topic.split('/')[-1]]=message
        arrays={}
        for camera in ['head','left','right']:
            clouds=[]
            for stamp in captures:
                frame=frames[(camera,stamp)];depth,mask,info=[frame[k] for k in ['depth','mask','info']]
                assert depth.encoding=='32FC1' and mask.encoding=='mono8' and not depth.is_bigendian
                z=np.ndarray((depth.height,depth.width),dtype='<f4',buffer=bytes(depth.data),strides=(depth.step,4))[::4,::4]
                selected=np.ndarray((mask.height,mask.width),dtype='u1',buffer=bytes(mask.data),strides=(mask.step,1))[::4,::4]>0
                selected &= np.isfinite(z)&(z>=.3)&(z<=3.)
                u,v=np.meshgrid(np.arange(0,depth.width,4),np.arange(0,depth.height,4))
                local=np.column_stack(((u[selected]-info.k[2])*z[selected]/info.k[0],(v[selected]-info.k[5])*z[selected]/info.k[4],z[selected]))
                transform=buffer.lookup_transform('base_link',depth.header.frame_id,Time(nanoseconds=stamp)).transform
                q=transform.rotation;x,y,zz,w=q.x,q.y,q.z,q.w
                rotation=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],
                    [2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]])
                t=transform.translation;clouds.append(local@rotation.T+[t.x,t.y,t.z])
            arrays[camera+'_first']=clouds[0];arrays[camera+'_all']=np.concatenate(clouds)
        np.savez_compressed(args.output/(label+'.npz'),**arrays)
        metadata['cases'][label]={'trial_index':index,'seed':trial['seed'],'code':trial['code'],
            'message':trial['result']['message'],'captures_ns':captures,'frames':trial['result']['accepted_frames'],
            'points':{key:len(value) for key,value in arrays.items()},'result':trial['result']}
    (args.output/'provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps({key:{k:v for k,v in item.items() if k!='result'} for key,item in metadata['cases'].items()},indent=2))


if __name__=='__main__':main()
