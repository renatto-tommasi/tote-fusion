# B2 same-byte ROS replay contract v3 — 2026-10-15

Starting revision: `3fd75de403ac19aee8fc934d23b58946e53ed1b2`.
The B1 criteria, generator, numerical workers and production node remain fixed.
The fresh core confirmation uses seeds200000–200199, conditions noise/shared_bias/
weak_complementary, both modes. Its source/configuration hashes are separate from
ROS adapter development. No hardware operation or model commit is authorized here.

The advisor `/root/benchmark_advisor` reviewed the native clock, partition, TF,
expiry and dropout interfaces before replay. This stage has these requirements:

- R01: Validate original cache/truth identities. Send the exact cached depth and
  mask bytes through production ROS Image/CameraInfo/TF/action interfaces. Truth
  stays in the evaluator. A fresh production node receives only an immutable
  model snapshot with the cached prior and original geometry file.
- R02: Retain two configured cameras (head/left), sigma0.003/0.006, minimumviews1,
  declared precision/Huber and stride4 across A/B/AB. Native DEGRADED_OK is an
  acceptance with explicit missing cameras. No production sampling or weight hook.
  Equal-sample and precision/loss ablations remain B1/core-only evidence.
- R03: Freeze named profile `paired_ros_v3`: collection1s, six acquisitions at
  release+0.05…0.55s, candidate validity2s, wall containment10s, maxframes12,
  maxpoints60000, reliable sensor QoS. Candidate validity explicitly differs from
  shipped0.5s expiry, before seeing results. Other fit/admission gates remain shipped.
- R04: Hold a positive constant ROS clock through fresh-node warmup and goal
  admission. Release only after accepted acknowledgement AND initial COLLECTING
  feedback, which follows native started_/end_ initialization. Then anchor continuous
  1x ROS time to release monotonic time throughout acquisition and estimation.
  No sensor acquisition occurs during the pause. Record frozen-wait duration,
  acknowledgement/feedback receipts, release anchors, clock cadence and lag.
  Include the entire pre-acquisition pause in primary wall send→result latency.
  This is pre-acquisition clock pause followed by1x acquisition/estimation replay,
  not a continuously1x whole action interval. No production instrumentation.
  Use a reliable explicit feedback observer with current DDS match count zero
  before the fresh node, and at least one match plus action readiness before
  sending. Supply the goal UUID and correlate COLLECTING to that exact UUID.
  ActionClient service discovery alone does not establish feedback delivery.
  Retain native result code/timings as a protocol-failure attempt if it precedes
  the feedback gate, separately from nominal estimator-performance summaries.
  Connection, scheduling, byte-identity or startup errors stop the protocol and
  remain distinct from estimator rejection. No automatic retries or guessed rows.
- R05: Record client-send→result and acknowledgement→result separately. Accepted
  rows retain candidate last_capture/issue_time, measured ROS age and mapped wall
  delivery age. End-to-end usability requires spatial completeness/accuracy,
  send→result<=3s and BOTH ages<=2s. Preserve late accepted results and rejections.
- R06: Normal moving control uses exact acquisition TF. Add a valid80ms bracket
  control with support poses on an explicitly defined piecewise geodesic path
  through the cached acquisition poses. Symmetric supports interpolate to the
  original cached pose, without relabelling a midpoint pose as historical support.
  Publish each future support causally, then deliver its delayed image triplet.
- R07: Wide-gap fault uses genuine cached moving-camera support poses separated
  by400ms, no exact sample for the faulted capture and no alternate static path.
  Publish the following support at its mapped time before delayed image delivery.
  An admitted A camera cannot mask an invalid participating B TF lookup.
- R08: Timestamp fault variants preserve pixel payloads while explicitly changing
  temporal metadata. Distinguish stale synchronized triplets from one-stream
  mismatch. Report actual native outside-window/pending/dropout behavior; do not
  claim every unmatched stream necessarily aborts a minimumviews1 request.
- R09: Retain every action result, rejected-frame count, admitted-camera counts,
  candidate publication count, selected input digests and lookup provenance.
  Fail the replay if a rejected action publishes a candidate or accepted result
  disagrees with its correlated candidate. Keep pilot and fresh studies separate.
- R10: Fresh-context independent B2 reviewer must reproduce appropriate positive
  and fault runs, challenge timing/TF/byte oracles, inspect the new confirmation
  identities/statistics and audit final evidence before closure.

First run one scene/mode/condition to validate transport, followed by30-scene
principal-condition ROS pilots if the protocol passes. Fresh200-scene numerical
confirmation is not a200-scene ROS or hardware study. The1% accepted-wrong target
cannot be qualified by200 zero-error accepted scenes. Editorial review and
ASD-STE100 language review remain separate from technical verification.

Protocol v1 is retained as a failed preflight, not a population result. It stopped
on a278ms acknowledgement before images because its25ms phase bound could not
establish native collection start. The robotics advisor selected v2 synchronization
before population execution. Spatial limits,3s wall latency and2s age remain fixed.
Both controls and all later populations must identify their protocol revision.
Protocol v2 passed24 control requests before missing its one-shot initial
feedback and stopping before acquisition. The v3 matched observer addresses
that readiness race. Both failed preflights remain preserved, without retries.
