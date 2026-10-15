#!/usr/bin/env python3
"""Same-byte ROS action replay; truth is read only after the node has returned."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from tote_fusion_interfaces.action import EstimateTote
from tote_fusion_interfaces.msg import TotePoseCandidate
from unique_identifier_msgs.msg import UUID
from metrics import score, proportion, quantiles
from scenes import scene_digest

ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ('head', 'left')


def ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def transform(matrix, optical, stamp):
    message = TransformStamped()
    message.header.frame_id = 'base_link'; message.child_frame_id = optical
    message.header.stamp = Time(nanoseconds=stamp).to_msg()
    p, q = message.transform.translation, message.transform.rotation
    p.x, p.y, p.z = map(float, matrix[:3, 3])
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
    return message


def summary(rows, criteria):
    groups = {}
    for row in rows:
        key = tuple(row[k] for k in ('partition', 'mode', 'condition', 'transport_variant', 'subset'))
        groups.setdefault(key, []).append(row)
    result = []
    for key, values in sorted(groups.items()):
        accepted = [v for v in values if v['accepted']]
        assert len({v['scene_id'] for v in values}) == len(values)
        result.append(dict(zip(('partition', 'mode', 'condition', 'transport_variant', 'subset'), key)) | {
            'requests': len(values), 'acceptance': proportion([v['accepted'] for v in values]),
            'spatial_usability': proportion([v['spatially_usable'] for v in values]),
            'end_to_end_usability': proportion([v['end_to_end_usable'] for v in values]),
            'accepted_wrong': proportion([v['accepted_wrong'] for v in accepted]),
            'wrong_all_requests': proportion([v['accepted_wrong'] for v in values]),
            'complete_required_geometry_accepted': proportion([not v['missing_required_axes'] for v in accepted]),
            'position_error_m': quantiles([v['position_error_m'] for v in accepted]),
            'orientation_error_rad': quantiles([v['orientation_error_rad'] for v in accepted]),
            'corner_error_m': quantiles([v['corner_error_m'] for v in accepted if v['corner_error_m'] is not None]),
            'dimension_error_m': {str(a): quantiles([v['dimension_errors_m'][str(a)] for v in accepted if str(a) in v['dimension_errors_m']]) for a in (1, 0)},
            'send_to_result_s': quantiles([v['send_to_result_s'] for v in values]),
            'wall_capture_to_result_s': quantiles([v['wall_age_s'] for v in accepted]),
            'ros_capture_to_issue_s': quantiles([v['age_s'] for v in accepted]),
            'coverage_95_joint': proportion([v['coverage']['0.95']['joint'] for v in accepted]),
            'codes': {c: sum(v['code'] == c for v in values) for c in sorted({v['code'] for v in values})}})
    paired = []; rng = np.random.default_rng(criteria['bootstrap_seed'])
    for key, values in sorted(groups.items()):
        if key[-1] != 'AB': continue
        fused = {v['scene_id']: v for v in values}
        ids = sorted(fused)
        for single in ('A', 'B'):
            other = {v['scene_id']: v for v in groups[key[:-1]+(single,)]}
            if fused.keys() != other.keys(): raise ValueError('ROS paired populations differ')
            draws = rng.integers(0, len(ids), (criteria['bootstrap_repetitions'], len(ids)))
            comparison = dict(zip(('partition', 'mode', 'condition', 'transport_variant'), key[:-1]))
            comparison['comparison'] = 'AB-'+single
            for metric in ('accepted', 'spatially_usable', 'end_to_end_usable', 'accepted_wrong'):
                delta = np.asarray([int(fused[i][metric])-int(other[i][metric]) for i in ids])
                comparison[metric+'_delta'] = {'mean': float(delta.mean()), 'paired_95_interval': list(map(float, np.quantile(delta[draws].mean(1), [.025, .975])))}
            paired.append(comparison)
    return {'stage': 'ros', 'criteria': criteria, 'groups': result, 'paired_comparisons': paired,
            'scope': 'Same-byte native ROS simulation, equal-time declared/Huber only; no hardware qualification'}


class Replay:
    def __init__(self, context):
        self.node = rclpy.create_node('paired_ros_evaluator', context=context, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor(context=context); self.executor.add_node(self.node)
        self.origin_wall = time.monotonic_ns(); self.origin_ros = 10**10; self.hold = None
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.dynamic = self.node.create_publisher(TFMessage, '/tf', 10)
        self.static = self.node.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pubs = [{topic: self.node.create_publisher(kind, f'/paired/{camera}/{topic}', 10)
                      for topic, kind in (('depth', Image), ('mask', Image), ('info', CameraInfo))} for camera in CAMERAS]
        self.received = {}
        self.image_subs = [self.node.create_subscription(Image, f'/paired/{name}/{kind}',
            lambda message, c=c, k=kind: self.received.__setitem__((c, k, ns(message.header.stamp)), hashlib.sha256(bytes(message.data)).hexdigest()), 10)
            for c, name in enumerate(CAMERAS) for kind in ('depth', 'mask')]
        self.published = []; self.sub = self.node.create_subscription(TotePoseCandidate, '/tote_fusion/candidate', self.published.append, 10)
        self.collecting = {}
        self.process = None; self.clock_gaps = []; self.last_tick = None; self.maximum_lag = 0

    def ros_now(self):
        return self.hold if self.hold is not None else self.origin_ros+time.monotonic_ns()-self.origin_wall

    def feedback(self, message):
        if message.feedback.state == 'COLLECTING':
            self.collecting.setdefault(bytes(message.goal_id.uuid), time.monotonic_ns())

    def tick(self):
        wall = time.monotonic_ns()
        if self.last_tick is not None:
            self.clock_gaps.append((wall-self.last_tick)/1e9)
        self.last_tick = wall
        self.clock.publish(Clock(clock=Time(nanoseconds=self.ros_now()).to_msg()))
        self.executor.spin_once(timeout_sec=.001)
        observed = self.node.get_clock().now().nanoseconds
        if observed:
            self.maximum_lag = max(self.maximum_lag, abs(self.ros_now()-observed)/1e9)
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError('Estimator process exited before replay completion')

    def wait(self, predicate, seconds=12):
        limit = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() > limit:
                raise TimeoutError('ROS replay protocol deadline')
            self.tick()

    def until(self, wall):
        self.wait(lambda: time.monotonic_ns() >= wall)

    def request(self, inputs, arrays, subset, variant, model_file, output):
        selected = (0, 1) if subset == 'AB' else (0,) if subset == 'A' else (1,)
        identity = inputs['scene_id']+'-'+subset+'-'+variant
        request_uuid = uuid.uuid4(); action_name = '/tote_fusion/estimate'
        request_object = identity+'-'+request_uuid.hex[:8]; self.collecting.clear()
        self.hold = self.ros_now()
        optical = [f'paired_{c}_{time.monotonic_ns()}' for c in range(2)]
        self.published.clear(); self.received.clear(); self.maximum_lag = 0; self.clock_gaps.clear(); self.last_tick = None
        model = yaml.safe_load(model_file.read_text()); prior = np.asarray(inputs['prior'])
        model['prior_pose'] = {'position_m': prior[:3, 3].tolist(), 'orientation_xyzw': Rotation.from_matrix(prior[:3, :3]).as_quat().tolist()}
        model['geometry_file'] = str((model_file.parent/model['geometry_file']).resolve())
        with tempfile.TemporaryDirectory(prefix='paired-ros-') as name:
            temp = Path(name); (temp/'model.yaml').write_text(yaml.safe_dump(model))
            profile = 'geometry_sim_pi_equivalent' if inputs['mode'] else 'multi_sim_pi_equivalent'
            parameters = {'use_sim_time': True, 'model_file': str(temp/'model.yaml'),
                'camera_file': str(model_file.parent/'cameras.yaml'), 'camera_ids': list(CAMERAS),
                'validation_profile': profile, 'reliable_sensor_qos': True, 'pixel_stride': 4,
                'candidate_validity_s': 2., 'wall_timeout_s': 10., 'max_frames': 12, 'max_points': 60000}
            for camera in range(2):
                prefix = '' if camera == 0 else 'left.'
                parameters[prefix+'optical_frame'] = optical[camera]
                parameters[prefix+'noise_sigma_m'] = inputs['configured_sigmas'][camera]
                for field, topic in (('depth', 'depth'), ('mask', 'mask'), ('camera_info', 'info')):
                    parameters[prefix+field+'_topic'] = f'/paired/{CAMERAS[camera]}/{topic}'
            (temp/'params.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            matches = [0]
            observer = self.node.create_subscription(EstimateTote.Impl.FeedbackMessage, action_name+'/_action/feedback', self.feedback, 10,
                event_callbacks=SubscriptionEventCallbacks(matched=lambda event: matches.__setitem__(0, event.current_count)))
            client = ActionClient(self.node, EstimateTote, action_name)
            self.wait(lambda: matches[0] == 0, 3)
            with (output/(identity+'.log')).open('x') as log:
                self.process = subprocess.Popen([str(ROOT/'build/tote_fusion/tote_fusion_node'), '--ros-args', '--params-file', str(temp/'params.yaml')], stdout=log, stderr=log)
                try:
                    self.wait(lambda: client.server_is_ready() and matches[0] > 0, 8)
                    fixed = []
                    for camera in selected:
                        frames = [f for f in inputs['frames'] if f['camera'] == camera]
                        if frames and not (inputs['condition'] == 'tf_gap' and camera == 1):
                            fixed.append(transform(np.asarray(frames[0]['body_camera']), optical[camera], 0))
                    self.static.publish(TFMessage(transforms=fixed)); self.until(time.monotonic_ns()+150_000_000)
                    goal = EstimateTote.Goal(object_id=request_object, expected_model_revision=0, min_distinct_views=1,
                        mode=inputs['mode'], validation_profile=profile)
                    goal.collection_duration.sec = 1
                    goal_uuid = UUID(uuid=list(request_uuid.bytes)); goal_key = bytes(goal_uuid.uuid)
                    sent_wall = time.monotonic_ns(); sent_ros = self.ros_now()
                    future = client.send_goal_async(goal, goal_uuid=goal_uuid); self.wait(future.done)
                    ack_wall = time.monotonic_ns(); handle = future.result()
                    if not handle.accepted: raise RuntimeError('Native node rejected benchmark goal before acquisition')
                    result_future = handle.get_result_async(); finished_times = []
                    result_future.add_done_callback(lambda _: finished_times.append(time.monotonic_ns()))
                    self.wait(lambda: goal_key in self.collecting or bool(finished_times))
                    if goal_key not in self.collecting:
                        native = result_future.result().result
                        attempt = {'classification': 'protocol_failure', 'goal_uuid': goal_key.hex(), 'object_id': identity,
                            'native_code': native.code, 'native_message': native.message, 'native_success': native.success,
                            'accepted_frames': native.accepted_frames, 'rejected_frames': native.rejected_frames,
                            'sent_wall_ns': sent_wall, 'ack_wall_ns': ack_wall, 'result_wall_ns': finished_times[0], 'acquisition_started': False}
                        (output/(identity+'-protocol-attempt.json')).write_text(json.dumps(attempt, indent=2)+'\n')
                        raise RuntimeError('Native result before matched COLLECTING gate: '+native.code)
                    released_wall = time.monotonic_ns(); self.origin_wall = released_wall; self.origin_ros = sent_ros; self.hold = None
                    self.maximum_lag = 0; self.clock_gaps.clear(); self.last_tick = released_wall
                    events = []; delivered = []
                    frames_b = [f for f in inputs['frames'] if f['camera'] == 1]
                    for frame in inputs['frames']:
                        camera, i = frame['camera'], frame['frame']
                        if camera not in selected:
                            continue
                        offset = frame['stamp_ns']-1_000_000_000; stamp = sent_ros+offset
                        matrix = np.asarray(frame['body_camera']); delivery = offset
                        if inputs['condition'] == 'tf_gap' and camera == 1:
                            if variant == 'wide_gap':
                                if i in (0, 4): events.append((offset, 0, ('tf', matrix, camera, stamp)))
                                delivery = max(offset, 460_000_000) if i else offset+10_000_000
                            elif variant == 'valid_bracket':
                                other = frames_b[i+1 if i < 5 else i-1]
                                seconds = (other['stamp_ns']-frame['stamp_ns'])/1e9
                                other_pose = np.asarray(other['body_camera'])
                                velocity = (other_pose[:3, 3]-matrix[:3, 3])/seconds
                                angular = Rotation.from_matrix(other_pose[:3, :3] @ matrix[:3, :3].T).as_rotvec()/seconds
                                for delta in (-.04, .04):
                                    support = matrix.copy(); support[:3, 3] += velocity*delta
                                    support[:3, :3] = Rotation.from_rotvec(angular*delta).as_matrix() @ matrix[:3, :3]
                                    shift = round(delta*1e9)
                                    events.append((offset+shift, 0, ('tf', support, camera, stamp+shift)))
                                delivery += 50_000_000
                            else:
                                events.append((offset, 0, ('tf', matrix, camera, stamp))); delivery += 10_000_000
                        events.append((delivery, 1, ('image', frame, stamp)))
                    for offset, _, event in sorted(events, key=lambda v: (v[0], v[1])):
                        self.until(released_wall+offset)
                        if result_future.done(): break
                        if event[0] == 'tf':
                            _, matrix, camera, stamp = event
                            self.dynamic.publish(TFMessage(transforms=[transform(matrix, optical[camera], stamp)]))
                        else:
                            _, frame, stamp = event; camera = frame['camera']
                            temporal = stamp-800_000_000 if variant == 'stale_triplet' and camera == 1 else stamp
                            header_stamp = Time(nanoseconds=temporal).to_msg()
                            info = CameraInfo(); info.header.stamp = header_stamp; info.header.frame_id = optical[camera]
                            info.width = frame['width']; info.height = frame['height']; info.distortion_model = 'plumb_bob'
                            fx, fy, cx, cy = (frame[k] for k in ('fx', 'fy', 'cx', 'cy'))
                            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
                            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                            info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
                            self.pubs[camera]['info'].publish(info)
                            digests = {}; message_stamps = {}
                            for kind, encoding, width_bytes in (('mask', 'mono8', 1), ('depth', '32FC1', 4)):
                                payload = arrays[frame['key']+'_'+kind].tobytes(); digests[kind] = hashlib.sha256(payload).hexdigest()
                                image = Image(); image.header.frame_id = optical[camera]
                                image.header.stamp = Time(nanoseconds=temporal).to_msg()
                                if variant == 'mask_skew' and camera == 1 and kind == 'mask':
                                    image.header.stamp = Time(nanoseconds=temporal+1).to_msg()
                                message_stamps[kind] = ns(image.header.stamp)
                                image.width = info.width; image.height = info.height; image.encoding = encoding
                                image.step = info.width*width_bytes; image.data = payload
                                self.pubs[camera][kind].publish(image)
                            delivered.append({'camera': camera, 'frame': frame['frame'], 'capture_ns': stamp, 'message_ns': temporal,
                                              'message_stamps_ns': message_stamps, 'payload_sha256': digests, 'wall_ns': time.monotonic_ns()})
                        self.tick()
                    self.wait(lambda: bool(finished_times)); finished_wall = finished_times[0]
                    reply = result_future.result().result
                    if reply.success: self.wait(lambda: len(self.published) == 1, 1)
                    else: self.until(time.monotonic_ns()+50_000_000)
                    if len(self.published) != int(reply.success): raise RuntimeError('Candidate publication/result disagreement')
                    self.wait(lambda: len(self.received) == len(delivered)*2, 1)
                    for item in delivered:
                        for kind in ('depth', 'mask'):
                            if self.received[(item['camera'], kind, item['message_stamps_ns'][kind])] != item['payload_sha256'][kind]:
                                raise RuntimeError('Received ROS pixel payload differs from cached bytes')
                    row = {'stage': 'ros', 'subset': subset, 'transport_variant': variant, 'budget': 'equal_time',
                        'precision': 'declared', 'loss': 'huber', 'accepted': reply.success, 'code': reply.code,
                        'message': reply.message, 'accepted_frames': reply.accepted_frames, 'rejected_frames': reply.rejected_frames,
                        'send_to_result_s': (finished_wall-sent_wall)/1e9, 'ack_to_result_s': (finished_wall-ack_wall)/1e9,
                        'ack_delay_s': (ack_wall-sent_wall)/1e9, 'compute_s': None, 'age_s': None, 'wall_age_s': None,
                        'pre_acquisition_pause_s': (released_wall-sent_wall)/1e9, 'collecting_feedback_wall_ns': self.collecting[goal_key],
                        'goal_uuid': goal_key.hex(), 'action_name': action_name, 'feedback_matched_publishers': matches[0],
                        'released_wall_ns': released_wall, 'released_ros_ns': sent_ros,
                        'sent_wall_ns': sent_wall, 'sent_ros_ns': sent_ros, 'ack_wall_ns': ack_wall, 'result_wall_ns': finished_wall,
                        'clock_anchor': {'monotonic_ns': self.origin_wall, 'ros_ns': self.origin_ros},
                        'maximum_clock_lag_s': self.maximum_lag, 'maximum_clock_publication_gap_s': max(self.clock_gaps),
                        'delivered': delivered, 'wire_payloads_verified': len(self.received), 'candidate_publications': len(self.published)}
                    if reply.success:
                        candidate = reply.candidate
                        if candidate != self.published[0]: raise RuntimeError('Published candidate fields differ from action result')
                        if candidate.object_id != request_object: raise RuntimeError('Candidate belongs to another request')
                        p, q = candidate.pose.pose.position, candidate.pose.pose.orientation
                        pose = np.eye(4); pose[:3, 3] = [p.x, p.y, p.z]
                        pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                        axes = [{'length': 0, 'width': 1}[name] for name in candidate.parameter_names]
                        n = 6+len(axes); dimensions = candidate.proposed_outer_dimensions_m
                        covariance = candidate.joint_covariance if inputs['mode'] else candidate.pose.covariance
                        row.update(pose=pose.tolist(), covariance=np.asarray(covariance).reshape(n, n).tolist(), dimension_axes=axes,
                            dimensions=[dimensions.x, dimensions.y, dimensions.z] if inputs['mode'] else [.6, .4, .23],
                            age_s=(ns(candidate.issue_time)-ns(candidate.last_capture))/1e9,
                            wall_age_s=(finished_wall-(self.origin_wall+ns(candidate.last_capture)-self.origin_ros))/1e9,
                            last_capture_ns=ns(candidate.last_capture), issue_ns=ns(candidate.issue_time),
                            source_camera_ids=list(candidate.source_camera_ids), missing_camera_ids=list(candidate.missing_camera_ids),
                            source_frame_counts=list(candidate.source_frame_counts), tf_lookup_stamps_ns=[ns(s) for s in candidate.tf_lookup_stamps],
                            tf_lookup_camera_ids=list(candidate.tf_lookup_camera_ids), candidate_id=candidate.candidate_id,
                            calibration_ids=list(candidate.calibration_ids), optical_frames=list(candidate.optical_frames))
                        row['covariance_calibrated'] = candidate.covariance_calibrated
                    return row
                finally:
                    self.process.send_signal(signal.SIGINT); exit_code = self.process.wait(timeout=5); self.process = None
                    if exit_code != 0: raise RuntimeError(f'Native node cleanup returned{exit_code}')
                    self.wait(lambda: not client.server_is_ready(), 3); client.destroy()
                    self.node.destroy_subscription(observer)


def fresh_request(inputs, arrays, subset, variant, model, output):
    transport = Context()
    lifecycle = {'scene_id': inputs['scene_id'], 'subset': subset, 'variant': variant, 'started_wall_ns': time.monotonic_ns()}
    try:
        with transport:
            replay = Replay(transport)
            lifecycle['ready_wall_ns'] = time.monotonic_ns()
            try:
                row = replay.request(inputs, arrays, subset, variant, model, output)
                row['context_lifecycle'] = lifecycle
            finally:
                lifecycle['teardown_started_wall_ns'] = time.monotonic_ns()
                stopped = replay.executor.shutdown(timeout_sec=2)
                replay.node.destroy_node()
                if not stopped: raise RuntimeError('Replay executor teardown timed out')
        return row
    finally:
        lifecycle.update(finished_wall_ns=time.monotonic_ns(), context_closed=not transport.ok())
        with (output/'contexts.jsonl').open('a') as events:
            events.write(json.dumps(lifecycle)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True); parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--conditions', nargs='+', required=True)
    parser.add_argument('--variants', nargs='+', choices=('normal', 'valid_bracket', 'wide_gap', 'stale_triplet', 'mask_skew'), default=['normal'])
    parser.add_argument('--domain', type=int, default=91)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    criteria = json.loads((ROOT/'benchmarks/criteria.json').read_text()); model = ROOT/'src/tote_fusion/config/tote_model.yaml'
    index = json.loads((args.cache/'index.json').read_text()); rows = []
    manifest = {'stage': 'ros', 'profile': 'paired_ros_v5', 'criteria': criteria, 'domain': args.domain,
        'cache_index_sha256': hashlib.sha256((args.cache/'index.json').read_bytes()).hexdigest(),
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__).resolve(), ROOT/'benchmarks/ros-contract.md', ROOT/'benchmarks/metrics.py', ROOT/'src/tote_fusion/src/node.cpp', *sorted((ROOT/'src/tote_fusion/config').glob('*'))]},
        'node_sha256': hashlib.sha256((ROOT/'build/tote_fusion/tote_fusion_node').read_bytes()).hexdigest(),
        'seeds': args.seeds, 'conditions': args.conditions, 'variants': args.variants}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (args.output/'ros_replay.source.py').write_bytes(Path(__file__).read_bytes())
    (args.output/'ros-contract.md').write_bytes((ROOT/'benchmarks/ros-contract.md').read_bytes())
    context = None
    try:
        with (args.output/'requests.jsonl').open('x') as records:
            entries = [e for e in index['scenes'] if int(e['scene_id'].rsplit('-', 1)[1]) in args.seeds and any('-'+c+'-' in e['scene_id'] for c in args.conditions)]
            if len(entries) != len(args.seeds)*len(args.conditions)*2: raise ValueError('Incomplete requested scene population')
            for entry in entries:
                folder = args.cache/entry['scene_id']; inputs = json.loads((folder/'inputs.json').read_text())
                with np.load(folder/'pixels.npz') as arrays:
                    if inputs['cache_digest'] != entry['cache_digest'] or scene_digest(inputs, arrays) != entry['cache_digest']: raise ValueError('Cache identity mismatch')
                    for variant in args.variants:
                        if variant in ('valid_bracket', 'wide_gap') and inputs['condition'] != 'tf_gap': continue
                        for subset in ('A', 'B', 'AB'):
                            context = {'scene_id': inputs['scene_id'], 'subset': subset, 'variant': variant}
                            row = fresh_request(inputs, arrays, subset, variant, model, args.output)
                            truth_bytes = (folder/'truth.json').read_bytes()
                            if hashlib.sha256(truth_bytes).hexdigest() != entry['truth_sha256']: raise ValueError('Truth identity mismatch')
                            row = score(row, inputs, json.loads(truth_bytes), criteria); row['tf_admission_exercised'] = True
                            row['end_to_end_usable'] = bool(row['spatially_usable'] and row['send_to_result_s'] <= criteria['maximum_latency_s'] and
                                row['age_s'] <= criteria['maximum_age_s'] and row['wall_age_s'] <= criteria['maximum_age_s']) if row['accepted'] else False
                            records.write(json.dumps(row, allow_nan=False)+'\n'); records.flush(); rows.append(row)
                            print(inputs['scene_id'], variant, subset, row['code'], flush=True)
        (args.output/'summary.json').write_text(json.dumps(summary(rows, criteria), indent=2, allow_nan=False)+'\n')
        manifest['artifacts'] = {str(p.relative_to(args.output)): hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file() and p.name != 'manifest.json'}
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    except Exception as error:
        (args.output/'protocol-failure.json').write_text(json.dumps({'context': context, 'type': type(error).__name__, 'message': str(error)}, indent=2)+'\n')
        raise
if __name__ == '__main__':
    main()
