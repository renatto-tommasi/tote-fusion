#!/usr/bin/env python3
"""Check independent timing, provenance and specified ROS control outcomes."""
import argparse
import json
from pathlib import Path


def check(rows):
    expected = {(mode, variant, subset) for mode in (0, 1) for variant in
                ('normal', 'valid_bracket', 'wide_gap', 'stale_triplet', 'mask_skew') for subset in ('A', 'B', 'AB')}
    indexed = {(r['mode'], r['transport_variant'], r['subset']): r for r in rows}
    assert len(rows) == len(indexed) == 30 and set(indexed) == expected
    for (mode, variant, subset), row in indexed.items():
        assert row['candidate_publications'] == int(row['accepted'])
        assert row['wire_payloads_verified'] == len(row['delivered'])*2
        assert row['feedback_matched_publishers'] == 1
        assert row['sent_ros_ns'] == row['released_ros_ns']
        assert row['sent_wall_ns'] <= row['collecting_feedback_wall_ns'] <= row['released_wall_ns']
        assert row['ack_wall_ns'] <= row['released_wall_ns']
        assert abs(row['send_to_result_s']-(row['result_wall_ns']-row['sent_wall_ns'])/1e9) < 1e-12
        assert abs(row['pre_acquisition_pause_s']-(row['released_wall_ns']-row['sent_wall_ns'])/1e9) < 1e-12
        for frame in row['delivered']:
            assert frame['capture_ns']-row['released_ros_ns'] in range(50_000_000, 551_000_000, 100_000_000)
            assert frame['wall_ns'] >= row['released_wall_ns']+frame['capture_ns']-row['released_ros_ns']
        if row['accepted']:
            assert sum(row['source_frame_counts']) == row['accepted_frames'] == len(row['tf_lookup_stamps_ns'])
            assert max(row['tf_lookup_stamps_ns']) == row['last_capture_ns']
            assert abs(row['age_s']-(row['issue_ns']-row['last_capture_ns'])/1e9) < 1e-12
            mapped = row['released_wall_ns']+row['last_capture_ns']-row['released_ros_ns']
            assert abs(row['wall_age_s']-(row['result_wall_ns']-mapped)/1e9) < 1e-12
        if variant in ('normal', 'valid_bracket') and (not mode or subset == 'AB'):
            assert row['accepted'] and row['spatially_usable'] and row['end_to_end_usable']
        if variant == 'wide_gap' and subset != 'A':
            assert not row['accepted'] and row['code'] == 'TF_UNAVAILABLE'
        if variant == 'stale_triplet' and subset != 'A':
            assert not row['accepted'] and row['code'] == 'STALE_OBSERVATION'
        if variant == 'mask_skew' and subset == 'B':
            assert not row['accepted'] and row['code'] == 'INSUFFICIENT_DATA'
        if variant == 'mask_skew' and subset == 'AB' and row['accepted']:
            assert row['code'] == 'DEGRADED_OK' and row['missing_camera_ids'] == ['left']
            assert row['source_camera_ids'] == ['head']
    print('30 ROS controls: causal clocks, payloads, candidates and fault outcomes passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    check([json.loads(line) for line in (args.results/'requests.jsonl').read_text().splitlines()])
