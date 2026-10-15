"""Independent finite-box RGB-D scene oracle. Never imported by the estimator."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

CONDITIONS = ('clean', 'noise', 'crop', 'occlusion', 'camera_bias',
              'shared_bias', 'weak_complementary', 'dropout', 'tf_gap', 'correct_prior')


def physical_boxes(dimensions):
    """Construct measured plate/handle/foot geometry, without estimator generators."""
    length, width, height = dimensions
    thickness = .0025
    boxes = [([-length/2, -width/2, .015], [length/2, width/2, .018])]
    for side in (-1, 1):
        y0, y1 = sorted((side*width/2, side*(width/2-thickness)))
        x0, x1 = sorted((side*length/2, side*(length/2-thickness)))
        boxes += [([-length/2, y0, .018], [length/2, y1, .185]),
                  ([-length/2, y0, .185], [-.055, y1, .223]),
                  ([.055, y0, .185], [length/2, y1, .223]),
                  ([x0, -width/2+thickness, .018], [x1, width/2-thickness, .223]),
                  ([-length/2, y0, .223], [length/2, y1, height]),
                  ([x0, -width/2+thickness, .223], [x1, width/2-thickness, height])]
        fy0, fy1 = sorted((side*(width/2-.003), side*(width/2-.0055)))
        fx0, fx1 = sorted((side*(length/2-.003), side*(length/2-.0055)))
        boxes += [([-length/2+.003, fy0, 0], [length/2-.003, fy1, .015]),
                  ([fx0, -width/2+.0055, 0], [fx1, width/2-.0055, .015])]
    result = np.asarray(boxes, dtype=float)
    assert result.shape == (17, 2, 3) and np.all(result[:, 1] > result[:, 0])
    return result


def look_at(origin, target):
    forward = target-origin
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(forward, right), forward))


def render(boxes, origin, rotation, width, height, focal, occluder=False):
    """Independent ray/slab first hit in tote coordinates, including openings."""
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack(((u-(width-1)/2)/focal, (v-(height-1)/2)/focal, np.ones_like(u)), -1)
    directions = rays @ rotation.T
    nearest = np.full((height, width), np.inf)
    selected = np.zeros((height, width), dtype=bool)
    objects = list(boxes)
    if occluder:
        objects.append(np.array([[-.42, -.18, .03], [-.34, .03, .31]]))
    for index, (low, high) in enumerate(objects):
        with np.errstate(divide='ignore', invalid='ignore'):
            a, b = (low-origin)/directions, (high-origin)/directions
        near, far = np.min(np.stack((a, b)), axis=0).max(-1), np.max(np.stack((a, b)), axis=0).min(-1)
        hit = (near > 0) & (far >= near) & (near < nearest)
        nearest[hit] = near[hit]
        selected[hit] = index < len(boxes)
    depth = np.where(np.isfinite(nearest), nearest, 2.9)
    points = origin+directions*depth[..., None]
    # The nominal estimator deliberately excludes labels/bevel/handle bands.
    selected &= (points[..., 2] > .027) & (points[..., 2] < .173) | (
        selected & (np.abs(points[..., 2]-.018) < 1e-7))
    selected &= (points[..., 2] < .079) | (points[..., 2] > .118)
    selected &= (depth >= .3) & (depth <= 3.)
    return depth, selected


def array_digest(arrays):
    digest = hashlib.sha256()
    for name in sorted(arrays):
        value = np.ascontiguousarray(arrays[name])
        digest.update(name.encode()); digest.update(value.dtype.str.encode())
        digest.update(str(value.shape).encode()); digest.update(value.tobytes())
    return digest.hexdigest()


def scene_digest(inputs, arrays):
    metadata = {key: value for key, value in inputs.items() if key != 'cache_digest'}
    return hashlib.sha256(json.dumps(metadata, sort_keys=True, allow_nan=False).encode()+array_digest(arrays).encode()).hexdigest()


def generate(destination, count, first_seed, partition, modes, conditions):
    destination.mkdir(parents=True, exist_ok=False)
    index = []
    for mode in modes:
        for condition in conditions:
            for trial in range(count):
                seed = first_seed+trial
                # Fixed named streams: adding/subsetting cameras cannot consume another stream.
                random = np.random.default_rng(np.random.SeedSequence([seed, mode, 0]))
                rotation = Rotation.from_euler('xyz', [random.uniform(-.01, .01), random.uniform(-.01, .01),
                                                        random.uniform(.35, .52)]).as_matrix()
                translation = np.array([1.02, 0., .5238])+random.uniform(-.015, .015, 3)
                truth = np.eye(4); truth[:3, :3] = rotation; truth[:3, 3] = translation
                dimensions = np.array([.6, .4, .23])
                if mode:
                    dimensions[:2] += random.uniform(.012, .025, 2)
                prior = truth.copy()
                if condition != 'correct_prior':
                    prior[:3, 3] += np.array([.020, -.004, .002])
                    prior[:3, :3] = Rotation.from_euler('z', .05235987756).as_matrix() @ rotation
                scene_id = f'{partition}-m{mode}-{condition}-{seed}'
                folder = destination/scene_id; folder.mkdir()
                arrays, frames = {}, []
                boxes = physical_boxes(dimensions)
                width, height, focal = (640, 480, 550.) if mode else (320, 240, 360.)
                for camera in range(2):
                    if condition == 'dropout' and camera == 1:
                        continue
                    origin = np.array([[-.95, 0., .65], [0., -.95, .65]][camera] if mode else
                                      [[-.9, -.65, 1.1], [-.65, -.9, 1.05]][camera])
                    origin += random.uniform(-.008, .008, 3)
                    fixed_optical = look_at(origin, np.array([0., 0., .08]))
                    if condition != 'tf_gap':
                        fixed_depth, fixed_mask = render(boxes, origin, fixed_optical, width, height, focal, condition == 'occlusion')
                    for frame in range(6):
                        physical_origin = origin.copy()
                        if condition == 'tf_gap' and camera:
                            physical_origin[0] += .04*(frame-2.5)
                        if condition == 'tf_gap':
                            optical = look_at(physical_origin, np.array([0., 0., .08]))
                            depth, mask = render(boxes, physical_origin, optical, width, height, focal)
                        else:
                            optical = fixed_optical
                            depth, mask = fixed_depth.copy(), fixed_mask.copy()
                        if camera == 0 and (condition == 'crop' or (condition == 'weak_complementary' and not mode)):
                            # Explicit observed ROI restriction, not a physical edge.
                            mask[:, :width//3] = False; mask[:, 2*width//3:] = False
                        if camera == 0 and condition == 'weak_complementary':
                            uu, vv = np.meshgrid(np.arange(width), np.arange(height))
                            rays = np.stack(((uu-(width-1)/2)/focal, (vv-(height-1)/2)/focal, np.ones_like(uu)), -1)
                            local_z = (physical_origin+(rays @ optical.T)*depth[..., None])[..., 2]
                            floor = np.abs(local_z-.018) < 1e-7
                            # Geometry: width wall without height support; pose: floor only.
                            mask &= ~floor if mode else floor
                        rng = np.random.default_rng(np.random.SeedSequence([seed, mode, camera+1, frame]))
                        if condition in ('noise', 'camera_bias', 'shared_bias'):
                            perturbation = rng.normal(0, .0015 if camera == 0 else .004, depth.shape)+rng.normal(0, .00075)
                            depth = np.where(mask, depth+perturbation, depth)
                        transform = np.eye(4); transform[:3, :3] = rotation @ optical
                        transform[:3, 3] = rotation @ physical_origin+translation
                        if condition == 'camera_bias' and camera == 1:
                            transform[:3, 3] += np.array([.015, -.008, .002])
                        if condition == 'shared_bias':
                            transform[:3, 3] += np.array([.010, .002, 0.])
                        key = f'c{camera}_f{frame}'
                        arrays[key+'_depth'] = depth.astype('<f4')
                        arrays[key+'_mask'] = (mask.astype('u1')*255)
                        arrays[key+'_tf'] = transform
                        frames.append({'key': key, 'camera': camera, 'frame': frame, 'stamp_ns': 1050000000+frame*100000000,
                                       'width': width, 'height': height, 'fx': focal, 'fy': focal,
                                       'cx': (width-1)/2, 'cy': (height-1)/2, 'body_camera': transform.tolist()})
                np.savez_compressed(folder/'pixels.npz', **arrays)
                meta = {'schema': 1, 'scene_id': scene_id, 'partition': partition, 'seed': seed, 'condition': condition,
                        'mode': mode, 'prior': prior.tolist(), 'frames': frames,
                        'configured_sigmas': [.003, .006], 'collection_duration_s': 1.,
                        'required_dimension_axes': [1, 0] if mode else [],
                        'ros_fault': 'wide_tf_gap' if condition == 'tf_gap' else None}
                digest = scene_digest(meta, arrays)
                meta['cache_digest'] = digest
                (folder/'inputs.json').write_text(json.dumps(meta, indent=2)+'\n')
                (folder/'truth.json').write_text(json.dumps({'pose': truth.tolist(), 'dimensions': dimensions.tolist()}, indent=2)+'\n')
                index.append({'scene_id': scene_id, 'cache_digest': digest, 'truth_sha256': hashlib.sha256((folder/'truth.json').read_bytes()).hexdigest()})
                print('cached', scene_id, flush=True)
    (destination/'index.json').write_text(json.dumps({'schema': 1, 'partition': partition, 'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'seed_span': [first_seed, first_seed+count-1], 'scenes': index}, indent=2)+'\n')
