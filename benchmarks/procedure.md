---
type: procedure
status: awaiting-review
created_date: 2026-10-15
updated_date: 2026-10-15
technical_verification: passed
verification_scope: Pixel-core pilot and benchmark checks only
ste_target: ASD-STE100 Issue 9
ste_language_review: pending
recall_tracking: excluded-procedure
---

# Run the paired pixel-core benchmark

## Purpose and applicability

Compare camera subsets with identical recorded pixels and model priors.
This procedure applies to the offline simulation benchmark in this repository.
It does not establish hardware tolerances, ROS transport performance or grasp safety.
The operating instructions target ASD-STE100 Issue 9. Full dictionary and language review remain pending.

## Prerequisites

- Use an assertions-enabled ROS 2 Jazzy build on Ubuntu 24.04.
- Install system NumPy, SciPy and PyYAML for Python 3.
- Reserve approximately 1 GB for each 600-scene cache and its results.
- Keep the estimator source and benchmark configuration fixed during a run.
- Use separate output paths for each run. Keep original evidence.

## Procedure

1. Open a terminal in the repository root.
2. Load the ROS environment.

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. Build the test executables.

   ```bash
   colcon build --packages-select tote_fusion_interfaces tote_fusion --symlink-install \
     --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
     '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g'
   ```

   Expected result: Both packages build without errors.

4. Run the benchmark checks.

   ```bash
   python3 -s -m unittest discover -s benchmarks -p 'test_benchmark.py' -v
   ```

   Expected result: Ten checks pass.

5. Generate the pilot cache in a new directory.

   ```bash
   mkdir -p benchmarks/runs
   python3 -s benchmarks/paired.py generate --output benchmarks/runs/pilot-cache \
     --scenes 30 --first-seed 100000 --partition pilot
   ```

   Expected result: `index.json` lists 600 scenes in 20 mode/condition groups.

6. Process the cache into a new result directory.

   ```bash
   python3 -s benchmarks/paired.py run --cache benchmarks/runs/pilot-cache \
     --output benchmarks/runs/pilot-results
   ```

   Expected result: `requests.jsonl`, `summary.json` and `manifest.json` contain the completed run.
   The result file contains 15,000 rows: 14,400 requests and 600 prior references.

7. Read the primary results for each condition.
8. Compare the paired intervals and uncertainty coverage.
9. Keep rejected and incomplete estimates in the report.
10. Read the scope limits in [the benchmark contract](contract.md).

## Failure handling and recovery

- Import error: Confirm that the command includes `python3 -s`.
  The tested system stack uses NumPy 1.26.4 and SciPy 1.11.4.
  Stop if that stack cannot import. Do not replace dependencies during a run.
- Existing output path: Select a new path. Do not delete original evidence to repeat a run.
- Cache identity error: Stop the run. Compare the original cache and manifest before regeneration.
- Estimator rejection: Read the explicit rejection code. Do not relax gates to improve acceptance.
- Interrupted run: Keep the partial output. Start a complete run in a new result directory.
  Resume and merge behavior are not implemented.
- Source or configuration change: Keep the old bundle. Rebuild and generate separately identified evidence.

## Completion

The manifest must identify the model, input cache, binaries, source and output artifacts.
An independent reviewer must reproduce the checks before milestone closure.
The numerical benchmark does not publish candidates or change installed production defaults.
No robot recovery action is part of this procedure.

## Project terminology

These technical nouns retain their software meanings. Dictionary review is pending.

| Term | Meaning |
| --- | --- |
| cache | Immutable scene inputs with a content digest |
| subset | Camera A, camera B or both cameras |
| prior | The supplied model pose before estimation |
| ablation | A controlled change to precision or robust loss |
| manifest | File identities and configuration for one run |
| paired interval | A bootstrap interval with the scene as the sampling unit |

## Related evidence

- [Contract](contract.md), [configuration](criteria.json), [implementation instructions](README.md)
- [Development session](session-20261015.md)
