# Independent B2 review — interim at 76ff1f3

Reviewer: `/root/verify_ros_comparison`, freshly spawned with no implementer conversation history. Scope: authorized offline CPU/ROS 2 Jazzy simulation, domain92. Production source, original binaries and original evidence were read-only. The review used a `git archive` of `76ff1f3e4758f61338138e117414f313a0322fae` and the identified original node/worker binaries through a read-only-use build symlink. Runtime and commands are recorded in `environment-and-actions.json`.

**Verdict: blocked at this revision.** A sequential fresh-server ROS action acknowledgement failure was independently reproduced. B2 populations and the fresh200 confirmation have not completed final evidence review. This is not an estimator-accuracy rejection or a hardware result.

## Confirmed finding B2-IV-01

- Requirement: R04 bounded request protocol; R10 independent final verification.
- Severity: medium, blocking acceptance-evidence/protocol defect.
- Revision/location:76ff1f3, `benchmarks/ros_replay.py:184`, repeated fresh production servers sharing `/tote_fusion/estimate`.
- Reproduction: `native-allconditions-v3.log` and `native-allconditions-v3/protocol-failure.json`;60 planned native requests,50 completed rows, request51 (`pilot-m1-weak_complementary-100000`, AB, normal) failed while awaiting the send-goal acknowledgement. Exit1. The native log reports `Failed to send goal response ... (timeout): client will not receive response`. This occurred after action readiness and matched feedback publisher checks. No acquisition loop was entered for the failed request.
- Expected: acknowledgement followed by exact-goal COLLECTING gate, or a recorded protocol failure excluded from estimator populations. The harness correctly stopped and retained failure, but the planned preflight could not complete and no principal population can be claimed from it.
- Observed historical corroboration: owner all-condition preflight stopped after39 rows at the same ACK wait, m1 occlusion A. Independently inspected original traceback and node log. The independent run passed that particular case before failing later, so failure is intermittent. A DDS/service-identity cause is an inference, not established middleware root cause.
- Disposition: open pending implementer remedy, a fresh identified preflight/population, and targeted independent reverification. Never merge or retry partial records into a nominal population.

## Reproduced passing evidence

- Thirty native tf_gap controls at exact76ff completed. Both modes, A/B/AB, normal/valid80ms bracket/wide400ms gap/stale synchronized triplets/one-stream1ns mismatch. `native-controls/manifest.json` identifies source, binary, criteria and all34 artifacts. Native wide-gap B and AB reject TF_UNAVAILABLE; stale B/AB reject STALE_OBSERVATION; B mask-skew rejects INSUFFICIENT_DATA. Accepted AB mask-skew pose is explicitly DEGRADED_OK with head only. Geometry A/B may reject their spatial gates; this is retained.
- The independently written `audit.py` reconstructs byte hashes directly from original NPZ caches, spatial position/rotation/pi-symmetric corners, eigenspace joint NEES with cross terms, covariance validity, camera/frame/TF lookup provenance, both clock ages and end-to-end classification. It passed all30 own controls, all30 historical controls,39 completed owner preflight rows, and50 completed independent preflight rows. This does not convert partial preflights into passing populations.
- All10 benchmark unit/integration tests passed (6.907s). The worker integration exercised both real precision binaries on2 scenes/50 rows and rejects tampered cache input. These checks are additional to native requests, not additional independent statistical scenes.

## Oracle challenges retained

- Actual isolated replay-source timing mutant changed send→result to ACK→result; native accepted, independent audit failed `send latency` (exit1).
- Actual isolated replay-source byte mutant altered image bytes before both publication and sender self-digest; native accepted, independent comparison to original cache failed `cached payload` (exit1). Sender/observer self-consistency alone would miss this regression.
- Actual isolated replay-source TF mutant translated valid-bracket support poses10mm. Independently captured actual TF publications were causal, but interpolation failed to reproduce original cached acquisition pose (exit1). Native also rejected CAMERA_CONFLICT; rejection alone was not the TF oracle.
- Restored valid-bracket source accepted and passed independent checks (exit0). Actual causal TF support timestamps,80ms bracket widths, interpolated translations and SLERP rotations were verified against cached acquisition transforms.
- Isolated metrics mutation discarded joint covariance cross terms in NEES; the independent eigenspace oracle failed `joint covariance score`; restored full covariance passed.
- Actual feedback callback accepted unrelated UUID into its dictionary without satisfying the requested UUID predicate. A nonempty-feedback predicate mutation was detected; restored exact-goal feedback satisfied readiness. This was a direct callback/predicate test, not a native wrong-UUID transport experiment.
- No production C++ source or sealed artifact was mutated. These are representative checks, not a universal mutation score.

## Provenance and remaining evidence

The original `/tmp/paired-ros-controls-v3` replay snapshot hash407be9… differs from76ff replay hash3dc66f… by candidate.object_id correlation validation and covariance_calibrated export. All34 historical artifacts match their manifest and their replay source matches its retained source snapshot. Those controls are traceable pre76ff evidence. Current coverage comes from the independently reproduced exact76ff controls; the historical bundle must not be relabelled exact76ff.

The running numerical confirmation manifest's17 listed source files match baseline5e890d2, and both numerical binaries/instrumented source match the original declared hashes. ROS adapter files were added after this manifest was created and are absent from its source list. There were no sealed result artifacts yet when inspected; the full1200-scene/30000-row population, cache identities, disjoint pilot seeds, all metrics/denominators and final summaries remain pending review. The principal ROS study has not started after the failed preflight. R10 is therefore incomplete independent of the protocol defect.

No claim of1% accepted-wrong qualification follows from200 accepted zero-error scenes; its one-sided95% bound exceeds1%. Simulation and oracle masks do not establish hardware calibration, deployment, production deadlines or a general fusion winner. Method timing shares a host and fixed execution order.

Read `benchmarks/procedure.md` for the applicable numerical operating instructions. That procedure does not cover the new ROS replay; owner ROS operating instructions remain pending. Technical state is partial/blocked for B2, editorial state awaiting-review, STE language review pending. The Development Session Workflow and portable handoff were read; project/vault closeout remains with the parent milestone owner after final evidence, avoiding concurrent project-note edits. No publication, push, hardware or controller operation occurred.
