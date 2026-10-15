"""Independent truth scoring and paired scene-level statistics."""
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import beta, chi2


def proportion(values, confidence=.95):
    count, total = sum(values), len(values)
    if not total:
        return {'count': 0, 'total': 0, 'fraction': None, 'interval': None, 'upper_one_sided': None}
    tail = (1-confidence)/2
    return {'count': int(count), 'total': total, 'fraction': count/total,
            'interval': [0. if count == 0 else float(beta.ppf(tail, count, total-count+1)),
                         1. if count == total else float(beta.ppf(1-tail, count+1, total-count))],
            'upper_one_sided': 1. if count == total else float(beta.ppf(confidence, count+1, total-count))}


def score(row, inputs, truth, criteria):
    row.update({name: inputs[name] for name in ('scene_id', 'seed', 'partition', 'mode', 'condition', 'cache_digest')})
    row['position_error_m'] = row['orientation_error_rad'] = row['joint_nees'] = row['corner_error_m'] = None
    row['dimension_errors_m'] = {}; row['coverage'] = {}; row['accepted_wrong'] = False
    row['spatially_usable'] = False; row['end_to_end_usable'] = None
    row['tf_admission_exercised'] = False
    for profile in ('primary', 'secondary'):
        row[profile+'_wrong'] = False
        row[profile+'_spatially_usable'] = False
    if not row['accepted']:
        return row
    estimated, actual = np.asarray(row['pose']), np.asarray(truth['pose'])
    symmetry = np.diag([-1., -1., 1.])
    alternatives = [actual[:3, :3], actual[:3, :3] @ symmetry]
    errors = [Rotation.from_matrix(r @ estimated[:3, :3].T).as_rotvec() for r in alternatives]
    chosen = min(range(2), key=lambda i: np.linalg.norm(errors[i]))
    error = list(actual[:3, 3]-estimated[:3, 3])+list(errors[chosen])
    row['symmetry_class'] = chosen
    row['position_error_m'] = float(np.linalg.norm(error[:3]))
    row['orientation_error_rad'] = float(np.linalg.norm(error[3:]))
    dimensions = np.asarray(row['dimensions']).reshape(3)
    for axis in row['dimension_axes']:
        delta = truth['dimensions'][axis]-dimensions[axis]
        error.append(delta); row['dimension_errors_m'][str(axis)] = abs(float(delta))
    covariance = np.asarray(row['covariance']); error = np.asarray(error)
    if covariance.shape != (len(error), len(error)) or not np.isfinite(covariance).all():
        raise ValueError('Invalid accepted covariance shape/values')
    if not np.allclose(covariance, covariance.T) or np.linalg.eigvalsh(covariance).min() <= 0:
        raise ValueError('Invalid accepted covariance definiteness')
    pose_nees = float(error[:6] @ np.linalg.solve(covariance[:6, :6], error[:6]))
    row['joint_nees'] = float(error @ np.linalg.solve(covariance, error))
    for confidence in (.90, .95, .99):
        row['coverage'][str(confidence)] = {'pose': bool(pose_nees <= chi2.ppf(confidence, 6)),
                                          'joint': bool(row['joint_nees'] <= chi2.ppf(confidence, len(error)))}
    row['missing_required_axes'] = sorted(set(inputs['required_dimension_axes'])-set(row['dimension_axes']))
    if not row['missing_required_axes']:
        corners = np.asarray(list(itertools.product((-1, 1), (-1, 1), (0, 1))))
        a = (corners*dimensions/[2, 2, 1]) @ estimated[:3, :3].T+estimated[:3, 3]
        b = (corners*np.asarray(truth['dimensions'])/[2, 2, 1]) @ alternatives[chosen].T+actual[:3, 3]
        row['corner_error_m'] = float(np.linalg.norm(a-b, axis=1).max())
    for profile in ('primary', 'secondary'):
        limits = criteria[profile]
        wrong = (row['position_error_m'] > limits['position_m'] or
                 row['orientation_error_rad'] > limits['orientation_rad'] or
                 any(e > limits['dimension_m'] for e in row['dimension_errors_m'].values()) or
                 (row['corner_error_m'] is not None and row['corner_error_m'] > limits['corner_m']))
        row[profile+'_wrong'] = bool(wrong)
        row[profile+'_spatially_usable'] = not wrong and not row['missing_required_axes']
    row['accepted_wrong'] = row['primary_wrong']
    row['spatially_usable'] = row['primary_spatially_usable']
    return row


def quantiles(values):
    return None if not values else dict(zip(('median', 'p95', 'maximum'), map(float, np.quantile(values, [.5, .95, 1.]))))


def summarize(rows, criteria):
    groups = {}
    for row in rows:
        if row['subset'] == 'prior':
            continue
        key = tuple(row[name] for name in ('stage', 'partition', 'mode', 'condition', 'budget', 'precision', 'loss', 'subset'))
        groups.setdefault(key, []).append(row)
    summaries = []
    for key, group in sorted(groups.items()):
        accepted = [r for r in group if r['accepted']]
        if len({r['scene_id'] for r in group}) != len(group):
            raise ValueError('Repeated scene would inflate independent counts')
        wrong = proportion([r['accepted_wrong'] for r in accepted])
        summaries.append(dict(zip(('stage', 'partition', 'mode', 'condition', 'budget', 'precision', 'loss', 'subset'), key)) | {
            'requests': len(group), 'acceptance': proportion([r['accepted'] for r in group]),
            'spatial_usability': proportion([r['spatially_usable'] for r in group]),
            'secondary_spatial_usability': proportion([r['secondary_spatially_usable'] for r in group]),
            'complete_required_geometry_accepted': proportion([not r['missing_required_axes'] for r in accepted]),
            'declared_axis_counts': {str(axis): sum(axis in r['dimension_axes'] for r in accepted) for axis in (1, 0)},
            'accepted_wrong': wrong, 'wrong_all_requests': proportion([r['accepted_wrong'] for r in group]),
            'false_acceptance_target': 'inconclusive' if not accepted else
                'met' if wrong['upper_one_sided'] <= criteria['maximum_accepted_wrong_fraction'] else
                'violated' if wrong['fraction'] > criteria['maximum_accepted_wrong_fraction'] else 'inconclusive',
            'position_error_m': quantiles([r['position_error_m'] for r in accepted]),
            'orientation_error_rad': quantiles([r['orientation_error_rad'] for r in accepted]),
            'dimension_error_m': {str(axis): quantiles([r['dimension_errors_m'][str(axis)] for r in accepted if str(axis) in r['dimension_errors_m']]) for axis in (1, 0)},
            'coverage_95_pose': proportion([r['coverage']['0.95']['pose'] for r in accepted]),
            'coverage_95_joint': proportion([r['coverage']['0.95']['joint'] for r in accepted]),
            'compute_s': quantiles([r['compute_s'] for r in group]),
            'compute_budget_exceeded': sum(r['compute_s'] > criteria['maximum_latency_s'] for r in group),
            'tf_fault_validation': 'not_exercised' if key[3] == 'tf_gap' else None,
            'rejections': {code: sum(r['code'] == code for r in group) for code in sorted({r['code'] for r in group if not r['accepted']})},
            'end_to_end_usability': None,
        })
    paired = []
    rng = np.random.default_rng(criteria['bootstrap_seed'])
    for key, fused in sorted(groups.items()):
        if key[-1] != 'AB':
            continue
        for single in ('A', 'B'):
            other = groups[key[:-1]+(single,)]
            left, right = {r['scene_id']: r for r in fused}, {r['scene_id']: r for r in other}
            if left.keys() != right.keys():
                raise ValueError('Paired methods have different scene populations')
            ids = sorted(left); draws = rng.integers(0, len(ids), (criteria['bootstrap_repetitions'], len(ids)))
            result = dict(zip(('stage', 'partition', 'mode', 'condition', 'budget', 'precision', 'loss'), key[:-1]))
            result['comparison'] = 'AB-'+single
            for metric in ('accepted', 'spatially_usable', 'accepted_wrong'):
                delta = np.asarray([int(left[i][metric])-int(right[i][metric]) for i in ids])
                result[metric+'_delta'] = {'mean': float(delta.mean()), 'paired_95_interval': list(map(float, np.quantile(delta[draws].mean(1), [.025, .975])))}
            common = [i for i in ids if left[i]['accepted'] and right[i]['accepted']]
            result['common_accepted_scenes'] = len(common)
            if common:
                delta = np.asarray([left[i]['position_error_m']-right[i]['position_error_m'] for i in common])
                samples = rng.integers(0, len(common), (criteria['bootstrap_repetitions'], len(common)))
                result['common_accepted_position_delta_m'] = {'median': float(np.median(delta)), 'paired_95_interval': list(map(float, np.quantile(np.median(delta[samples], axis=1), [.025, .975])))}
            paired.append(result)
    prior_groups = {}
    for row in rows:
        if row['subset'] == 'prior':
            prior_groups.setdefault((row['partition'], row['mode'], row['condition']), []).append(row)
    priors = [dict(zip(('partition', 'mode', 'condition'), key)) | {
        'scenes': len(group), 'position_error_m': quantiles([r['position_error_m'] for r in group]),
        'orientation_error_rad': quantiles([r['orientation_error_rad'] for r in group]),
        'scope': 'Unchanged regularizer/model reference, not an accepted sensor estimate'} for key, group in sorted(prior_groups.items())]
    return {'criteria': criteria, 'groups': summaries, 'prior_reference': priors, 'paired_comparisons': paired,
            'scope': 'Pixel-core simulation; no measured ROS/action age or end-to-end usability. No hardware or task superiority claim.'}
