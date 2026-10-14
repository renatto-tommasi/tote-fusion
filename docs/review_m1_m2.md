# Independent M1/M2 review — 2026-10-14

This fresh-context review inspected the sealed M1/M2 source, interfaces, adapters,
tests, configuration, corpus and mutation bundles before Phase 2. It applied
`AGENTS.md`, the specified robust-minimal-code skill, the engineering specification
and design decisions. No production source or sealed evidence was changed by the
reviewer. The implementation owner is resolving findings separately; the locations
below refer to the original sealed M2 snapshot and may move during remediation.

**Initial disposition: two blocking M2 findings.** The existing acceptance suite
passes, but does not detect a received camera disappearing from quality validation
or excessive dynamic-TF interpolation gaps. M1's stationary simulation evidence
remains credible within its declared scope. Resolve and independently verify both
findings before treating M2 as reviewed or proceeding to M3.

## Findings

### R1 — High: unsupported received camera can count toward successful coverage

Location: `src/tote_fusion/src/core.cpp:333–345` in the sealed M2 source;
`src/tote_fusion/src/node.cpp:319–325` and `:362–365`.

Per-camera checks iterate `train.cameras`, populated only when a ray produces a
supported model correspondence. A camera present in both raw partitions can have
zero correspondences and disappear from this map. Equal train/test map sizes do
not detect its absence. `camera_rmse` starts at zero; the ROS adapter still counts
the camera's accepted acquisition frames, satisfies the goal minimum, and includes
it in candidate provenance with a misleading zero held-out RMSE. Enough samples
from a good camera satisfy the aggregate inlier gate.

An independent compiled fixture used the existing nondegenerate analytic head
observations and 40 additional fitting/40 held-out rays from camera 1 whose body
coordinates lie outside the tote. The original implementation returned:

```text
ACCEPTED cameras=2 bad_camera_rmse=0 inliers=0.78
```

This violates the documented requirement that every participating received camera
have independent fitting/held-out support. It can certify a two-camera request
using only one supported view. It is not permitted missing-camera degradation.

Remediation: derive participating camera IDs from raw inputs; require minimum
supported correspondences in both partitions for every such ID before calculating
its residual. Reject unsupported received views explicitly and publish no candidate.
Add core and ROS regressions where one received camera has no model support and
the remaining camera passes all rank/quality gates. Also cover one-sided support.
Demonstrate that bypassing the new check restores the erroneous acceptance.

### R2 — High: acquisition-time TF lookup has no dynamic-support gap gate

Location: `src/tote_fusion/src/node.cpp:239–243` in the sealed M2 source.

The acquisition stamp is correctly supplied to TF2 and latest TF is never used.
However, successful lookup alone does not enforce the specification's section 5
maximum-age/gap checks. TF2 interpolates between available transforms, even when
their separation is much longer than the validated moving-wrist fixture. There
is no checked dynamic-TF support-gap configuration or rejection path in this
implementation. The 80 ms bracketing test establishes numerical interpolation
for its fixture, not admissibility of a sparse transform history.

An independent real TF2 Buffer check supplied dynamic transforms at 1 s and 9 s.
A capture-time lookup at 5 s succeeded and interpolated x=4 m across the 8 s gap.
Thus a fresh depth sample can be transformed using unsupported long interpolation
and published without identifying that uncertainty. Static transforms need no
recent publication and must remain exempt.

Remediation: define an explicit simulation maximum interpolation/support gap;
validate dynamic transform support for every edge of the acquisition-time chain,
including any dynamic body/world edge, before accepting the frame. Reject excessive
gaps rather than guessing motion. Add a nonidentity moving-camera sparse-gap test
with fresh sensor stamps, plus a normal short-bracket and static-transform control.
If a narrower TF contract is chosen, enforce it at the actual input boundary and
document the exercised scope; documentation alone does not implement the gate.

### R3 — Medium: reproduction commands overwrite sealed evidence

Location: `docs/m2_evaluation.md:140–141` in the sealed M2 source.

The commands redirect fresh evaluator output directly into
`verification/final_m2/noise-two.csv` and `noise-three.csv`. Timing columns differ
on rerun, so even identical numerical outcomes invalidate sealed artifact hashes.
The nearby instruction to use a new directory contradicts these runnable examples.

Remediation: create a new output directory and route all regenerated CSV/corpus
files there. Keep the final manifest and old evidence immutable, and create a
separate remediation bundle whose source/configuration hashes identify the fixes.

### R4 — Low, follow-up: rejection diagnostics omit useful quality evidence

Location: `src/tote_fusion/src/node.cpp:255–273` and
`src/tote_fusion/src/core.cpp:350–359` in the sealed M2 source.

Diagnostics expose code, revision, aggregate counts, queue bytes, last capture and
per-camera accepted counts/calibration IDs. Accepted candidates carry rank and
residuals. Rejected requests do not retain numerical weak-mode/rank, per-camera
support/residual or request-correlation evidence in diagnostics. This limits the
section 8 operability contract: an alive node repeatedly rejecting views is harder
to diagnose than the action code alone suggests.

Remediation: retain concise request-associated quality evidence at the rejection
boundary and expose the relevant observed support/rank/residual fields. Avoid
per-pixel logs or a new framework. This is a nonblocking operability follow-up for
the simulation scope, not evidence of an unsafe accepted candidate by itself.

### R5 — High: a processed brief clock reset can publish an in-flight candidate

Found during remediation review. Location: original sealed
`src/tote_fusion/src/node.cpp:343–346`; TF remediation's `observe_clock()` at
`src/tote_fusion/src/node.cpp:199–205` and `poll()`.

Reset detection samples `now()` in callbacks and a 20 ms timer. A backward jump
followed by catch-up between these samples is not observable from the later
clock value. This can mix request epochs and fails the specified rule that a ROS
time jump invalidates the in-flight request. The new bounded TF history also
needs clearing on such an event.

The independent ROS test accepts six head/left captures, pauses its heartbeat,
publishes clock zero, obtains a rejected overlapping goal with a `BUSY` diagnostic
whose header stamp is zero, then immediately restores the clock and closes
collection. The zero-stamped diagnostic proves the estimator processed clock
zero; this is not merely a dropped clock message. Trial 0 detected reset, but
trial 1 published **`DEGRADED_OK`** and a candidate. The named assertion fails
with `Processed backward clock jump escaped polling and published DEGRADED_OK`.
Source, failing XML and actual logs are retained in
`verification/review_m1_m2/brief-reset-original.*` and
`brief_reset_independent.py`.

Remediation: register a ROS clock-jump notification for actual backward updates.
Signal a reset event safely across the clock/executor boundary; discard dynamic
TF support and reject the active request before any further admission/publication.
Do not mutate executor-owned maps or strings unsafely from a clock callback.
Retain the short-pulse oracle, its original failure, and restored rejection.
R5 is blocking until independently verified; the long-zero reset test does not
distinguish this failure.

## Independent checks and evidence

- Verified all **281 M1** and **592 M2** final-manifest artifact hashes; none were
  missing or changed. Before implementation-owner remediation began, all **56**
  current-source hashes matched the M2 handoff.
- Re-ran `source setup_env.bash` and `ctest --test-dir build/tote_fusion
  --output-on-failure`: core, action and multi-view wrappers passed, approximately
  21.65 s total. These were the original built M2 targets; source remediation was
  not assumed tested by this run.
- Re-ran segmentation, planner-model and evaluator Python tests: **64 passed**.
  An initial command used nonexistent `src/tote_planning/test`, ran no tests and
  exited 4; the corrected path was `src/tote_planning_model/tests`.
- Decoded every **18,255** message in the release MCAP through `rosbag2_py` and
  ROS deserialization. Depth dimensions/encoding/payload and binary masks matched
  the declared 160×120 fixture. Every **1,734** accepted candidate lookup
  `(camera, acquisition stamp)` pair had its corresponding retained depth sample.
  All 125 request records consistently distinguish success from empty candidate.
- Recomputed both 60-request workload latency quantiles and code counts directly
  from the corpus trial records; they match the summary. Negative controls retain
  the expected conflict, invalid-metadata, unobservable, canceled and reset codes.
- Recomputed accepted-row covariance symmetry, positive definiteness and NEES
  from the full CSV error/covariance matrices; maximum NEES differences were below
  `8e-15`. Independently reran the two-/three-camera noise executables: all **420
  rows per camera workload** matched every retained nontiming field exactly.
  Fresh CSVs were written outside sealed evidence.
- Inspected actual mutation sources, failing XML and restoration reports. M1's
  five named transform/staleness/ambiguity/unit/revision mutants fail their
  numerical or action oracles. M2's newest-TF mutant fails moving and interpolated
  TF tests; the conflict-gate bypass fails the named camera-conflict oracle.
  This review inspected the retained compiled-mutant evidence and did not rerun
  all isolated mutations.
- Source/dependency inspection supports the truth boundary: the estimator links
  the pure core, ROS sensor/action/TF dependencies and interfaces, not simulator or
  evaluator code. Image-derived segmentation has no truth import. Private truth
  querying and candidate-to-world conversion occur in test/evaluator code. Retained
  live and replay graph audits allow only documented sensor, clock, TF and ROS
  parameter inputs; no runtime estimator truth query was found.
- Immutable model/configuration, capture-time lookup provenance, strict symmetry
  rejection or explicit pi-yaw class, data-only information rank, bounded point/
  frame/payload queues, finite solver iterations, atomic cancellation checks and
  steady wall deadlines are implemented with meaningful existing oracles. No
  guessed-data fallback, planner commit, robot-control path, or unjustified runtime
  framework was found. Camera-specific weighting, loss covariance comparison and
  the explicit shared-bias limitation oracle are appropriate within their scope.

Review reproductions and fresh numerical reruns are retained in
`verification/review_m1_m2/`. The sealed snapshots retain the exact original
source locations. A later remediation review must record new hashes and evidence
without rewriting these original findings.

## Limits

This review did not rerun Gazebo scene bringup, full 125-request acquisition, a
fresh Release/NDEBUG rebuild or all mutation builds. It inspected their retained
artifacts and ran the focused tests above. Historical M1 tests were reviewed in
their sealed snapshot; the fresh tests execute the M2 implementation. Analytic
replay motion is prismatic with nonidentity fixed orientations, not time-varying
wrist rotation, actuated Gazebo motion or hardware motion. Finite accepted
covariance samples, conditional pi-yaw uncertainty, small images and logical
queue instrumentation do not establish physical calibration, production tail
timing, arbitrary shared-bias detection or downstream model-manager correctness.
Those limitations are stated honestly in the milestone evaluation.

## Remediation verification

R1 core correction independently verified: the implementation now enumerates
camera IDs present in raw inputs and requires supported fitting and held-out
counts for each. Recompiling the original independent reproduction against the
rebuilt core returns `INSUFFICIENT_DATA` instead of acceptance. Original and fixed
logs are retained separately. A fresh focused `Multiview.*` core run passes all
10 tests, including the new zero-support regression. The reviewer independently
reran the ROS multi-view wrapper, which passes in 15.67 s including rejection for
unrelated depth from a received second camera. R1 is resolved in core and ROS.
R2 resolution remains pending at this checkpoint.

R2 subsequently passed independent source inspection and the reviewer’s isolated
domain-90 ROS suite: 11 active tests and clean shutdown, 16.602 s. The actual
8-second dynamic-ancestor gap rejects despite independent TF2 interpolation
working; normal 80 ms brackets, exact samples and static transforms pass.
Used-chain static/topology/classification/conflicting-timestamp mutations reject.
The bounded snapshot uses the same public-TF2 records whose support is checked,
without a listener/metadata race. The configured body frame must be an ancestor
of each camera in the supported topology; other trees explicitly reject.

R3's commands now create a fresh `mktemp` output directory. R4 is improved:
per-camera support/residual/inlier/bias and data-only rank/eigenvalues occur in
rejection messages forwarded to action results and diagnostics. Dedicated
machine-readable quality fields and request correlation remain a low-priority
operability limit. **Final review closure remains blocked by R5** at this
checkpoint.

R5 independently resolved in the subsequent clock-jump correction. The unchanged
original `brief_reset_independent.py` now passes all five trials: every trial
retains a BUSY diagnostic at clock zero as processing proof, returns
`SIM_TIME_RESET`, publishes no candidate, and shuts down cleanly. The fixed
log/XML are separate from the original failing evidence. This targeted run took
7.446 s including the five trials.

The retained pre-jump handler signals atomic reset/cancellation flags; executor
code consumes the event with `exchange(false)` and clears dynamic TF support.
The callback performs no clock reads or map/string mutation. Candidate assembly
and time validation occur outside the publication mutex. Only the final reset
check and typed candidate publication occur inside it; action completion,
diagnostics and cleanup occur after release. The jump handler's member lifetime
ends before the mutex and atomic fields are destroyed. No clock-lock inversion
or unchecked reset-to-publication window was found in this source inspection.
The publication commit is the completion boundary; a later jump invalidates
subsequent requests rather than retroactively invalidating a finished candidate.

**Final implementation disposition: no remaining blocking review findings for
M1/M2's declared simulation scope.** R1, R2, R3 and R5 are resolved with the
independent checks above. R4's remaining machine-readable/correlation detail is
a nonblocking operability limit. The implementation owner must still retain
updated meaningful mutation results, the post-remediation workload/source
snapshot and its handoff. Existing sealed milestone hashes and the original
failed reproductions must remain preserved. This review does not extend the
claims to hardware, arbitrary TF trees, time-varying wrist rotation, production
calibration/timing or model-manager commits.
