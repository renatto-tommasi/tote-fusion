---
type: development-session
status: awaiting-review
session_date: 2026-10-15
timezone: America/Mexico_City
repository: https://github.com/renatto-tommasi/tote-fusion
starting_commit: 3fd75de403ac19aee8fc934d23b58946e53ed1b2
implementation_commit: c1702adcae00381de21ff35250a171401403887f
technical_verification: passed
verification_scope: B2 isolated native ROS replay and fresh numerical confirmation
ste_language_review: pending
---

# B2 native ROS comparison and fresh confirmation

## Goal and scope

Extend the [reviewed B1 pixel-core comparison](session-20261015.md) through the
production RGB-D, mask, TF and action interfaces. Run a separate fresh numerical
confirmation without changing the estimator, criteria or numerical configuration.
The user delegated picking-limit and consequential protocol decisions to a robotics
expert. The [ROS contract](ros-contract.md) records R01–R10 and protocol revisions.

This record follows the knowledge vault's Development Session Workflow and Codex
Session Handoff. It is a repository record. No new vault notes or recall ledger
edits occurred. Editorial approval remains awaiting review. The linked
[procedure](ros-procedure.md) targets ASD-STE100 Issue 9, has language review pending
and is excluded from recall tracking. B1's historical closeout remains unchanged.

## Implementation

- ros_replay.py validates cached input/truth identities and supplies the original
  depth/mask bytes, CameraInfo and acquisition-time transforms to a fresh native
  process. Independent parent observers hash the received Image payloads. Truth
  stays outside the native process and is scored after its result.
- A/B/A+B keep the original two-camera catalog and declared sigmas. The native
  solver, quality gates, sources, headers and installed defaults remain at the
  original production baseline 434803c. The ROS profile explicitly uses one-second
  collection, twelve-frame capacity and two-second candidate validity; the
  shipped validity default is 0.5 seconds. This is a simulation replay profile.
- Each request has its own rclpy Context, SingleThreadedExecutor, evaluator node,
  native process and ROS entities. Only the explicit executor spins. Context
  startup and teardown are exported separately from action latency.
- The evaluator waits for a matched feedback observer, action readiness, accepted
  response and initial COLLECTING for the supplied exact goal UUID. It holds a
  positive clock before acquisition, then maps clock time to monotonic wall time
  at one times speed throughout acquisition and estimation. The admission pause
  is included in send-to-result latency. This is not continuous-time replay for
  the whole action interval.
- Six cached captures are scheduled at release+0.05 through release+0.55 seconds.
  Positive moving TF uses exact transforms or valid 80 ms interpolation brackets
  on a defined geodesic path through the cached poses. Following transforms and
  delayed image delivery are causal. The 400 ms fault gap uses genuine distinct
  cached support poses, with no exact replacement transform.
- Stale triplets and a one-nanosecond mask mismatch preserve original pixels.
  The reports retain native rejection codes, frame counts and missing cameras.
  Minimum-views-one behavior can accept A with missing B; it is not B success.
- Action/candidate equality is checked by ROS message fields and correlated object
  identity. Raw serialized CDR bytes are not a semantic equality oracle. UUID,
  candidate identity, lookup times, image digests, raw state, full covariance,
  monotonic/ROS clock anchors, latency and both estimate ages remain in each row.
- check_ros_controls.py checks thirty positive/fault outcomes and temporal,
  publication, payload and lifecycle invariants. A protocol exception stops the
  run and preserves its attempt. No failed request is retried or replaced.

## Expert decisions

The expert retained the frozen provisional primary criteria: 5 mm position,
1 degree orientation, 3 mm per required dimension, 15 mm corner displacement,
3 seconds send-to-result, 2 seconds latest-capture age and a 1% accepted-wrong
target with a one-sided exact 95% upper bound. These are simulation proxies.
Actual gripper clearance, robot speed and sensor/calibration distributions remain
unknown. Secondary limits never replace primary failures.

The advisor selected explicit collection-start synchronization before population
execution, then fresh parent contexts after fixed-count diagnostics showed that
unique action names did not remove response timeouts. Fresh contexts measure
isolated requests. Zero failures in forty screening attempts has a one-sided
95% upper failure-rate bound of approximately 7.2%; it is not a reliability claim.
The advisor required preservation of the production baseline and separate
documentation of the admission-recovery finding below.

## Failed attempts and reconciled evidence

These attempts remain separate from the completed populations. No stopped run
supplies missing rows to another run. Local originals and their source snapshots
are retained; compact public evidence is linked in the final results record.

| Attempt | Observation and treatment |
| --- | --- |
| ROS smoke v1 | Semantic candidate/result fields agreed, but CDR byte comparison failed. Retained and excluded. |
| ROS smoke v2 / v3 probe | Six requests each completed. The probe demonstrated differing CDR encodings with equal deserialized fields. Diagnostic only; intermediate source provenance is explicit. |
| Controls v1 | Twelve rows completed; request thirteen stopped before images on a 278 ms ACK that exceeded the proposed 25 ms phase bound. Protocol replaced before population use. |
| Controls v2 | Twenty-four rows completed; the one-shot initial feedback was missed. Stopped before acquisition. The attempt did not retain the actual native code, a documented evidence limitation. |
| Controls v3 | Thirty controls passed. Its source predates 76ff1f3 by the object-identity assertion and calibrated-covariance export; it is not relabelled as that commit. |
| Normal preflight at 76ff1f3 | Thirty-nine rows completed, then goal acknowledgement timed out before image delivery. Native log reports response-send failure. Independent reproduction stopped on request fifty-one after fifty rows. Original B2 review was blocked. |
| Reused-name diagnostic | Forty no-image goal/cancel attempts: 37 CANCELED, 3 ACK timeouts. Preserve all attempts and responsive parameter heartbeats. |
| Unique-name diagnostic | Forty attempts: 35 CANCELED, 5 ACK timeouts. Name isolation did not resolve the observed failure; DDS root cause remains unproven. |
| First fresh-context diagnostic | Forty setup failures, zero goals: Context.__enter__ already initializes the context. An extra init call was erroneous. Invalid arm; no transport-rate inference. |
| Corrected fresh-context diagnostic | Forty CANCELED, zero failures, forty contexts closed. Screening passed; no sensor-performance result. |
| Diagnostic helper generation | A temporary generated helper had a SyntaxError before ROS execution. Failed helper source/log remain retained and excluded. |

The old blocked review and the later remedy report remain distinct. The final
source c1702ad removes the redundant identity name remap and uses the canonical
action endpoint. Later controls and populations identify this exact revision.

## Separate production availability finding

The installed rclcpp_action version is 28.1.21
(28.1.21-1noble.20260615.145316). Its library SHA256 is
4a90f28eff2b8af478befa6987f018a2cb7f6f379d429ab0f068b7eb1e5e3cc7.
The advisor compared the installed library with the
[matching upstream source](https://github.com/ros2/rclcpp/blob/28.1.21/rclcpp_action/src/server.cpp).
On goal-response RCL_RET_TIMEOUT, the server returns before the accepted callback.
The node sets reserved_ in its goal callback, but initializes goal_ and the wall
deadline only in the accepted callback. Its poll returns immediately without goal_.
That code path leaves a reservation without the normal wall watchdog.

Response-send timeouts were dynamically observed. Persistent subsequent BUSY was
not dynamically reproduced. The source-level admission-recovery gap is therefore
reported separately from the reproduced transport symptom. Fresh parent contexts
do not repair the native path. No broader bounded-admission-recovery qualification
or validated automatic restart recovery is claimed. Production remediation and
a direct injected-timeout regression are future work outside this frozen study.

## Verification status

**B2 closed within the approved simulation benchmark scope.** The fresh-context
[final independent report](evidence/b2-20261015/review/final-report.md) and its
[38-file seal](evidence/b2-20261015/review/final-manifest.json) leave no blocker for
this comparison stage. This verdict evaluates the provisional targets; it does
not certify task accuracy, calibrated confidence, hardware or general availability.

- Passed: owner thirty controls and sixty all-condition preflight requests at
  c1702ad, with ninety closed contexts and sealed manifests.
- Passed: fresh-context reviewer independently repeated those ninety requests on
  domain 92. Cache-byte, TF, time, state/covariance, UUID and context oracles passed.
- Passed: reviewer mutations detected omitted admission latency, changed payload,
  biased interpolation supports, removed cross covariance and wrong goal UUID.
  Restored checks passed. The original blocked report is retained.
- Passed: reviewer audited all 1,200 fresh confirmation cache identities and
  separation from pilot seeds. Cache index SHA256 is
  df501cf8272299b9c1cf6a4b55994f3c7142e73769ac92e9dd54267c669de059.
- Passed: numerical confirmation completed with exit zero: 30,000 rows comprise
  28,800 sensor requests and 1,200 prior references. The reviewer independently
  audited all 2,402 output hashes, seventeen baseline source hashes, both worker
  binaries, every accepted state/corner/full-covariance score, priors, null scores,
  factor selection identities, exact binomial intervals and 96 paired comparisons.
  The 144 sensor cells each contain 200 scenes. [Results](results-confirmation-20261015.md)
  include the independent corner supplement and retain every ablation in raw output.
- Passed: principal ROS pilot completed with exit zero: 180 scenes, 540 requests,
  thirty seeds per condition/mode, every context closed. All requests met the
  3-second goal-latency limit (maximum 1.5154 seconds). Accepted results met both
  2-second age limits. The [results](results-confirmation-20261015.md) retain the
  complete outcomes and the explicit timing/profile scope.
- Passed: independent final ROS audit verified all 545 output artifacts, eight
  source identities, binary/cache hashes, 540 raw states/byte/time/TF/UUID records,
  closed contexts, eighteen thirty-scene cells and twelve paired comparisons.
  Public compressed rows and supporting archives match the originals exactly.
  The final tables/prose and original blocked/remedy seals were also rechecked.
- Passed: the robotics expert's [advisory documentation review](evidence/b2-20261015/robotics-review.md)
  approved the amended README. Four required guidance corrections qualify the
  execution watchdog, separate ACK/BUSY escalation, restrict historical lifecycle
  claims and scope the earlier review. This advisory review did not rerun studies.
- Not run: 200-scene-per-condition ROS confirmation, real segmentation/sensors,
  hardware grasps, continuous-client reliability and production admission recovery.

## Lessons and failure modes

| Lesson or trigger | Detection and response |
| --- | --- |
| Action readiness does not prove initial feedback delivery | Require a currently matched reliable observer and the exact UUID COLLECTING gate. |
| ACK receipt is later than native collection start | Establish collection initialization before clock release; include the pause in wall latency. |
| Continuous parent context can retain problematic transport state | Preserve fixed-count diagnostics. Use the tested isolation protocol without claiming a root cause or continuous-client reliability. |
| Context context-manager already initializes itself | Use one lifecycle owner and explicit executor; retain setup failures instead of counting them as native requests. |
| Serialized bytes can differ for equal ROS messages | Check generated field equality for candidates; hash actual image payload bytes for input identity. |
| Convenient TF supports can change the physical observation | Construct a defined causal path and independently verify interpolation to each cached acquisition pose. |
| A dropped stream differs from invalid participating TF | Report actual missing-camera admission separately from TF/timestamp rejection. |
| Two biased cameras can agree | Use independent truth; acceptance and ellipsoid coverage do not establish picking accuracy. |
| Candidate can be accurate but too old or too late | Require both ages and full send-to-result latency for end-to-end usability. |
| Protocol failure or missing final manifest | Stop and retain the incomplete run. Start any revised experiment in a new directory. |
| A later finding narrows an old guarantee | Correct the operational instruction and recovery row, not only an appended limitations section. Preserve historical reviews with their original scope. |

## Handoff

Implementation: [c1702ad](https://github.com/renatto-tommasi/tote-fusion/commit/c1702adcae00381de21ff35250a171401403887f),
preceded by [76ff1f3](https://github.com/renatto-tommasi/tote-fusion/commit/76ff1f3e4758f61338138e117414f313a0322fae).
These implementation revisions are pushed with this evidence/documentation
record on main. The final chat handoff reports the verified remote commit.
The [ROS procedure](ros-procedure.md) supplies exact commands and failure handling.
The [B1 pilot record](results-20261015.md) remains a separate population.
The [results and complete evidence links](results-confirmation-20261015.md) retain
all request populations, summaries, manifests and independent reviews.
The [outer checksum index](evidence/b2-20261015/SHA256SUMS) covers the compact
publication without changing any sealed inner artifact. No hardware deployment,
model commit or new vault publication occurred.
