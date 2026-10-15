"""Independent geometry/statistics and real worker integration checks."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from scenes import render, physical_boxes, scene_digest, generate
from metrics import score, proportion, summarize
from paired import run, ROOT

CRITERIA = json.loads((ROOT/'benchmarks/criteria.json').read_text())


class GeometryAndStatistics(unittest.TestCase):
    def test_first_hit_is_front_not_back(self):
        depth, _ = render(np.array([[[-.5, -.5, -.5], [.5, .5, .5]]]), np.array([0., 0., -2.]), np.eye(3), 3, 3, 10.)
        self.assertEqual(depth[1, 1], 1.5)

    def test_handle_is_open_not_an_infinite_wall(self):
        optical = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
        boxes = physical_boxes([.6, .4, .23])
        opening, _ = render(boxes, np.array([0., -1., .204]), optical, 3, 3, 100.)
        wall, _ = render(boxes, np.array([.1, -1., .204]), optical, 3, 3, 100.)
        self.assertEqual(opening[1, 1], 2.9)
        self.assertAlmostEqual(wall[1, 1], .8)

    def test_cache_identity_covers_prior_and_pixels_but_not_truth(self):
        arrays = {'depth': np.ones((2, 2), dtype='<f4')}
        inputs = {'prior': np.eye(4).tolist()}; initial = scene_digest(inputs, arrays)
        inputs['prior'][0][3] = .01
        self.assertNotEqual(initial, scene_digest(inputs, arrays))
        inputs['prior'][0][3] = 0
        arrays['depth'][0, 0] = 2
        self.assertNotEqual(initial, scene_digest(inputs, arrays))

    def row(self):
        return {'accepted': True, 'pose': np.eye(4).tolist(), 'dimensions': [[.6], [.4], [.23]],
                'dimension_axes': [], 'covariance': (np.eye(6)*1e-5).tolist(), 'compute_s': .01,
                'stage': 'pixel_core', 'subset': 'AB', 'precision': 'declared', 'loss': 'huber', 'budget': 'equal_time', 'code': 'OK'}

    def inputs(self):
        return {'scene_id': 'scene', 'seed': 100000, 'partition': 'pilot', 'mode': 0, 'condition': 'clean',
                'cache_digest': 'identity', 'required_dimension_axes': []}

    def truth(self):
        return {'pose': np.eye(4).tolist(), 'dimensions': [.6, .4, .23]}

    def test_pi_equivalent_pose_scores_zero_with_matching_corners(self):
        row = self.row(); row['pose'][0][0] = -1; row['pose'][1][1] = -1
        result = score(row, self.inputs(), self.truth(), CRITERIA)
        self.assertEqual(result['orientation_error_rad'], 0)
        self.assertEqual(result['corner_error_m'], 0)
        self.assertTrue(result['spatially_usable'])
        self.assertIsNone(result['end_to_end_usable'])

    def test_joint_cross_covariance_is_used(self):
        row = self.row(); row['dimension_axes'] = [1]; row['dimensions'][1][0] = .397
        row['pose'][0][3] = -.003
        covariance = np.eye(7)*1e-6; covariance[0, 6] = covariance[6, 0] = .5e-6
        row['covariance'] = covariance.tolist()
        result = score(row, self.inputs(), self.truth(), CRITERIA)
        self.assertAlmostEqual(result['joint_nees'], 12., places=8)
        self.assertNotAlmostEqual(result['joint_nees'], 18.)

    def test_missing_required_axis_is_incomplete_not_zero_error(self):
        inputs = self.inputs(); inputs['mode'] = 1; inputs['required_dimension_axes'] = [1, 0]
        result = score(self.row(), inputs, self.truth(), CRITERIA)
        self.assertFalse(result['spatially_usable']); self.assertFalse(result['accepted_wrong'])
        self.assertEqual(result['dimension_errors_m'], {}); self.assertIsNone(result['corner_error_m'])

    def test_missing_accepted_evidence_fails(self):
        row = self.row(); del row['pose']
        with self.assertRaises(KeyError):
            score(row, self.inputs(), self.truth(), CRITERIA)

    def test_false_acceptance_bound_does_not_invent_evidence(self):
        self.assertGreater(proportion([False]*200)['upper_one_sided'], .01)
        self.assertLess(proportion([False]*299)['upper_one_sided'], .01)
        self.assertIsNone(proportion([])['fraction'])

    def test_paired_statistics_retain_rejections_and_scene_identity(self):
        rows = []
        for i in range(8):
            for variant in ('A', 'B', 'AB'):
                row = self.row(); row['subset'] = variant
                row['accepted'] = variant == 'AB'
                inputs = self.inputs(); inputs['scene_id'] = str(i)
                rows.append(score(row, inputs, self.truth(), CRITERIA))
        summary = summarize(rows, CRITERIA)
        for comparison in summary['paired_comparisons']:
            self.assertEqual(comparison['accepted_delta']['mean'], 1.)
            self.assertEqual(comparison['spatially_usable_delta']['paired_95_interval'], [1., 1.])
            self.assertEqual(comparison['common_accepted_scenes'], 0)
        rows.append(copy.deepcopy(rows[0]))
        with self.assertRaises(ValueError):
            summarize(rows, CRITERIA)


class WorkerIntegration(unittest.TestCase):
    def test_same_cached_pixels_and_budgets_across_real_ablations(self):
        with tempfile.TemporaryDirectory(prefix='paired-test-') as name:
            directory = Path(name); cache = directory/'cache'; output = directory/'results'
            generate(cache, 1, 100321, 'pilot', [0, 1], ['clean'])
            run(cache, output, ROOT/'build/tote_fusion', ROOT/'src/tote_fusion/config/tote_model.yaml', CRITERIA)
            rows = [json.loads(line) for line in (output/'requests.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 50)
            for mode in (0, 1):
                current = [r for r in rows if r['mode'] == mode and r['subset'] != 'prior']
                self.assertEqual(len({r['cache_digest'] for r in current}), 1)
                for budget in ('equal_time', 'equal_samples'):
                    for subset in ('A', 'B', 'AB'):
                        variants = [r for r in current if r['budget'] == budget and r['subset'] == subset]
                        self.assertEqual(len({r['selected_data_sha256'] for r in variants}), 1)
                        if budget == 'equal_samples':
                            counts = {(r['fitting_points'], r['held_points'], r['fitting_boundaries'], r['held_boundaries'])
                                      for r in current if r['budget'] == budget}
                            self.assertEqual(len(counts), 1)
                fused = [r for r in current if r['subset'] == 'AB' and r['loss'] == 'huber' and r['precision'] == 'declared']
                self.assertTrue(all(r['accepted'] for r in fused))
            # A changed cached pixel must stop replay, even though its shape remains valid.
            folder = cache/json.loads((cache/'index.json').read_text())['scenes'][0]['scene_id']
            with np.load(folder/'pixels.npz') as source:
                arrays = dict(source)
            arrays['c0_f0_depth'][0, 0] += .01
            np.savez_compressed(folder/'pixels.npz', **arrays)
            with self.assertRaisesRegex(ValueError, 'Cached input'):
                run(cache, directory/'tampered', ROOT/'build/tote_fusion', ROOT/'src/tote_fusion/config/tote_model.yaml', CRITERIA)


if __name__ == '__main__':
    unittest.main()
