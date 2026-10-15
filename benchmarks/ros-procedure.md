---
type: procedure
status: awaiting-review
created_date: 2026-10-15
updated_date: 2026-10-15
technical_verification: passed
verification_scope: Isolated native ROS comparison requests only
ste_target: ASD-STE100 Issue 9
ste_language_review: pending
recall_tracking: excluded-procedure
---

# Run the cached ROS comparison

## Purpose and prerequisites

Replay identical cached pixels through the production action, image and TF interfaces.
Use this procedure after the [pixel-core procedure](procedure.md).
The instructions target ASD-STE100 Issue 9. Dictionary and language review remain pending.

- Use the assertions-enabled Jazzy build and the system Python scientific stack.
- Keep the source, executable, criteria and cache fixed during a run.
- Reserve an unused ROS domain. The owner used 91. The reviewer used 92.
- Use new output directories. Keep failed and partial runs.
- Read the [ROS contract](ros-contract.md) before the first request.

## Procedure

1. Open a terminal in the repository root.
2. Load the build environment.

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

3. Set the cache path to the completed 600-scene pilot cache.

   ```bash
   cache="$PWD/benchmarks/runs/pilot-cache"
   ```

   Expected result: The directory contains index.json and the original scene files.
   The [pixel-core procedure](procedure.md) creates this cache.

4. Run the positive and fault controls in a new directory.

   ```bash
   python3 -s benchmarks/ros_replay.py --cache "$cache" \
     --output benchmarks/runs/ros-controls --seeds 100000 --conditions tf_gap \
     --variants normal valid_bracket wide_gap stale_triplet mask_skew --domain 91
   python3 -s benchmarks/check_ros_controls.py benchmarks/runs/ros-controls
   ```

   Expected result: Thirty requests complete. The control checker passes.
   A rejection is an expected result for the specified fault controls.

5. Run the normal one-scene preflight for all conditions.

   ```bash
   python3 -s benchmarks/ros_replay.py --cache "$cache" \
     --output benchmarks/runs/ros-preflight --seeds 100000 \
     --conditions clean noise crop occlusion camera_bias shared_bias \
       weak_complementary dropout tf_gap correct_prior --domain 91
   ```

   Expected result: Sixty requests complete. Each request has a closed context record.
   Do not require all estimator requests to be accepted.

6. Run the principal-condition pilot.

   ```bash
   python3 -s benchmarks/ros_replay.py --cache "$cache" \
     --output benchmarks/runs/ros-principal --seeds {100000..100029} \
     --conditions noise shared_bias weak_complementary --domain 91
   ```

   Expected result: 540 requests complete for 180 scenes.
   The directory contains requests.jsonl, contexts.jsonl, summary.json and a final manifest.json.

7. Check that the final manifest contains output artifact hashes.
8. Read acceptance, spatial accuracy, complete geometry and end-to-end usability separately.
9. Compare A+B with each camera on paired scenes.
10. Keep the numerical confirmation and ROS pilot populations separate.
11. Obtain an independent evidence review before closing B2.

## Timing and configuration

The paired_ros_v5 profile uses a one-second collection and a two-second candidate
validity interval. The shipped validity interval is 0.5 seconds. This benchmark
profile does not change the installed default.

Each request creates a new Python context, executor and evaluator node, plus a
new production process. The canonical action name remains fixed. The evaluator
holds a positive ROS clock during admission. It waits for the accepted response
and COLLECTING feedback for the exact goal UUID. It then advances ROS time at
one times wall time during acquisition and estimation. No image is acquired
during the pause. The reported goal latency includes the full pause.

Context startup and teardown are separate measurements. These results apply to
isolated requests. They do not establish reliability for a persistent client.
Both ROS capture-to-issue age and mapped wall capture-to-result age must pass.

## Failure handling

- Protocol error: Stop. Keep protocol-failure.json, logs and partial rows.
  Do not retry the failed request or replace it in the population.
- Identity error: Stop. Compare the original cache and source hashes.
  Do not edit input files to make the check pass.
- Missing final artifact hashes: Treat the run as incomplete.
- Expected estimator rejection: Keep the native code and diagnostics.
  A rejected request must not publish a candidate.
- Accepted estimate: Check its object identity, candidate/result field equality,
  required observable dimensions, independent errors, latency and both ages.
- Context teardown failure: Stop. Do not continue with surviving ROS entities.
- Goal-response timeout: Preserve the native log and protocol-failure.json.
  A dedicated protocol-attempt.json is saved only when a native result arrives
  before the initial feedback gate.
  Fresh contexts reduced this failure in the screening experiment. They do not
  repair the production reservation path described in the session record.
  Do not claim a validated automatic recovery procedure for that fault.

The wide TF gap and stale triplet controls must reject participating B inputs.
A one-stream timestamp mismatch has a different meaning. A minimum-views-one
request can accept A with explicit missing-B diagnostics. Do not label that
outcome as a successful B observation.

## Completion and limits

The manifest identifies the cache, source, native executable and output files.
Independent observers check the received depth and mask bytes against the cache.
The truth files are read by the evaluator after the action result.

Oracle masks, simulated depth, fixed viewpoints, a pre-acquisition clock pause,
modified candidate validity and isolated processes limit transfer to deployment.
The procedure sends no robot motion or model commit command.

Related records: [session](session-ros-20261015.md), [criteria](criteria.json),
[contract](ros-contract.md), [pixel-core pilot](results-20261015.md).
