#!/usr/bin/env python3
"""Cache once, run paired pixel-core subsets/ablations, score independently."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
import yaml
from scenes import CONDITIONS, scene_digest, generate
from metrics import score, summarize

ROOT = Path(__file__).resolve().parents[1]


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def run(cache, output, build, model, criteria):
    output.mkdir(parents=True, exist_ok=False)
    index = json.loads((cache/'index.json').read_text())
    source_paths = [*sorted((ROOT/'src/tote_fusion/src').glob('*.cpp')),
                    ROOT/'src/tote_fusion/include/tote_fusion/core.hpp', *sorted((ROOT/'benchmarks').glob('*.py')),
                    ROOT/'benchmarks/worker.cpp', ROOT/'benchmarks/contract.md', ROOT/'benchmarks/criteria.json',
                    ROOT/'src/tote_fusion/CMakeLists.txt', *sorted((ROOT/'src/tote_fusion/config').glob('*'))]
    manifest = {'stage': 'pixel_core', 'cache': str(cache.resolve()), 'source_sha256': {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths},
        'binaries': {name: hashlib.sha256((build/f'benchmark_{name}').read_bytes()).hexdigest() for name in ('declared', 'equal')},
        'instrumented_core_sha256': hashlib.sha256((build/'equal_precision_core.cpp').read_bytes()).hexdigest(),
        'criteria': criteria, 'base_commit': '434803c4bc30c4d8f759dc8409f13e7bfdfce063',
        'model_file': str(model.resolve()), 'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
        'cache_index_sha256': hashlib.sha256((cache/'index.json').read_bytes()).hexdigest(),
        'scope': 'Pixel projection/boundary extraction + numerical estimator. No ROS goal/TF validation or measured estimate age.'}
    dump(output/'manifest.json', manifest)
    rows = []
    with (output/'requests.jsonl').open('x') as records:
        for entry in index['scenes']:
            folder = cache/entry['scene_id']; inputs = json.loads((folder/'inputs.json').read_text())
            truth = json.loads((folder/'truth.json').read_text())
            if inputs['cache_digest'] != entry['cache_digest']:
                raise ValueError('Cache index/input identity mismatch')
            if hashlib.sha256((folder/'truth.json').read_bytes()).hexdigest() != entry['truth_sha256']:
                raise ValueError('Truth artifact hash mismatch')
            with np.load(folder/'pixels.npz') as arrays:
                if scene_digest(inputs, arrays) != inputs['cache_digest']:
                    raise ValueError('Cached input bytes or metadata changed')
                with tempfile.TemporaryDirectory(prefix='tote-paired-') as name:
                    temporary = Path(name)
                    (temporary/'inputs.json').write_text((folder/'inputs.json').read_text())
                    for frame in inputs['frames']:
                        for kind in ('depth', 'mask'):
                            key = frame['key']+'_'+kind
                            (temporary/(key+'.bin')).write_bytes(arrays[key].tobytes())
                    for precision in ('declared', 'equal'):
                        result = temporary/(precision+'.yaml')
                        command = [str((build/f'benchmark_{precision}').resolve()), str(temporary), str(model.resolve()), str(result)]
                        with (output/(entry['scene_id']+'-'+precision+'.log')).open('x') as log:
                            subprocess.run(command, stdout=log, stderr=log, check=True)
                        original = yaml.safe_load(result.read_text())
                        if len(original) != 12:
                            raise ValueError('Incomplete worker result matrix')
                        for row in original:
                            if row['scene_id'] != inputs['scene_id'] or row['cache_digest'] != entry['cache_digest']:
                                raise ValueError('Worker input identity mismatch')
                            row = score(row, inputs, truth, criteria)
                            records.write(json.dumps(row, allow_nan=False)+'\n'); records.flush(); rows.append(row)
            # The prior is an uncommitted reference, not a sensor acceptance.
            base = yaml.safe_load(model.read_text()); covariance = np.asarray(base['prior_covariance']['matrix'])
            position = np.asarray(inputs['prior'])[:3, 3]
            x, y, z = position; conversion = np.eye(6)
            conversion[:3, 3:] = -np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            reference = {'stage': 'pixel_core', 'subset': 'prior', 'budget': 'reference', 'precision': 'prior', 'loss': 'none',
                         'accepted': True, 'code': 'PRIOR_REFERENCE', 'pose': inputs['prior'], 'dimension_axes': [],
                         'dimensions': [[.6], [.4], [.23]], 'covariance': (conversion @ covariance @ conversion.T).tolist(),
                         'compute_s': 0., 'age_s': None}
            reference = score(reference, inputs, truth, criteria)
            reference.update(accepted=None, accepted_wrong=None, spatially_usable=None, end_to_end_usable=None)
            reference['reference_dimension_errors_m'] = {str(a): abs(truth['dimensions'][a]-[.6, .4, .23][a]) for a in (1, 0)}
            records.write(json.dumps(reference, allow_nan=False)+'\n'); rows.append(reference)
            print('replayed', entry['scene_id'], flush=True)
    dump(output/'summary.json', summarize(rows, criteria))
    manifest['artifacts'] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sorted(output.iterdir()) if p.is_file() and p.name != 'manifest.json'}
    dump(output/'manifest.json', manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    create = commands.add_parser('generate')
    create.add_argument('--output', type=Path, required=True)
    create.add_argument('--scenes', type=int, default=30)
    create.add_argument('--first-seed', type=int, required=True)
    create.add_argument('--partition', choices=('pilot', 'confirmation'), required=True)
    create.add_argument('--modes', type=int, nargs='+', choices=(0, 1), default=[0, 1])
    create.add_argument('--conditions', nargs='+', choices=CONDITIONS, default=list(CONDITIONS))
    execute = commands.add_parser('run')
    execute.add_argument('--cache', type=Path, required=True); execute.add_argument('--output', type=Path, required=True)
    execute.add_argument('--build', type=Path, default=ROOT/'build/tote_fusion')
    execute.add_argument('--model', type=Path, default=ROOT/'src/tote_fusion/config/tote_model.yaml')
    execute.add_argument('--criteria', type=Path, default=ROOT/'benchmarks/criteria.json')
    args = parser.parse_args()
    if args.command == 'generate':
        low, high = (100000, 200000) if args.partition == 'pilot' else (200000, 300000)
        if not (1 <= args.scenes <= 10000 and low <= args.first_seed and args.first_seed+args.scenes <= high):
            raise ValueError('Use disjoint pilot [100000,200000) and confirmation [200000,300000) seeds')
        generate(args.output, args.scenes, args.first_seed, args.partition, args.modes, args.conditions)
    else:
        run(args.cache, args.output, args.build, args.model, json.loads(args.criteria.read_text()))


if __name__ == '__main__':
    main()
