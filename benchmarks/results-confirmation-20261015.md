# Fresh numerical confirmation and native ROS pilot — 2026-10-15

Fusion helps when views supply missing geometry, but this study does not establish
a general fusion advantage. Noisy pose has a measurable acceptance/accuracy tradeoff;
noisy geometry remains unusable. Shared calibration bias produces accepted-wrong
poses even when the reported uncertainty region covers truth.

The expert-selected [criteria](criteria.json) remain provisional simulation proxies:
5 mm position, 1 degree orientation modulo pi, 3 mm per required dimension,
15 mm corner displacement, 3 s action latency, 2 s age and at most 1% wrong among
accepted estimates. No criteria, production solver or gates were tuned after results.
[Core contract](contract.md), [ROS contract](ros-contract.md),
[session](session-ros-20261015.md), [procedure](ros-procedure.md).

## Fresh numerical population

Seeds 200000–200199 are separate from the pilot. There are 1,200 scenes:
200 per condition/mode for noise, shared_bias and weak_complementary, in pose and
geometry modes. The same frozen B1 workers process all four independent
precision/loss settings and both sample budgets. The 30,000 rows comprise
28,800 sensor requests and 1,200 unchanged-prior references. A scene is the
independent unit. Do not pool conditions or count frames/methods as extra trials.
All segmentation masks are oracle test masks.

These tables use declared precision and Huber loss. Counts use denominator 200.
Accepted-only errors do not describe rejected scenes. Missing a required dimension
makes an estimate incomplete; an accepted incomplete estimate can be neither
spatially wrong nor usable. The raw records preserve that distinction.

## Equal capture time

| Mode | Condition | Usable A | Usable B | Usable A+B | Accepted A+B | Wrong/accepted A+B | 95% joint coverage A+B | A+B position p95 (mm) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Pose | noise | 200 | 194 | 194 | 194 | 0/194 | 194/194 | 0.684 |
| Pose | shared_bias | 0 | 0 | 0 | 195 | 195/195 | 195/195 | 10.470 |
| Pose | weak_complementary | 0 | 199 | 200 | 200 | 0/200 | 200/200 | 0.033 |
| Geometry | noise | 0 | 0 | 0 | 0 | 0/0 | — | — |
| Geometry | shared_bias | 0 | 0 | 0 | 0 | 0/0 | — | — |
| Geometry | weak_complementary | 0 | 0 | 200 | 200 | 0/200 | 200/200 | 0.335 |

## Equal raw sample control

| Mode | Condition | Usable A | Usable B | Usable A+B | Accepted A+B | Wrong/accepted A+B | 95% joint coverage A+B | A+B position p95 (mm) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Pose | noise | 197 | 194 | 194 | 194 | 0/194 | 194/194 | 0.853 |
| Pose | shared_bias | 0 | 0 | 0 | 194 | 194/194 | 194/194 | 10.675 |
| Pose | weak_complementary | 0 | 200 | 200 | 200 | 0/200 | 200/200 | 0.105 |
| Geometry | noise | 0 | 0 | 0 | 1 | 0/1 | 1/1 | 4.101 |
| Geometry | shared_bias | 0 | 0 | 0 | 1 | 1/1 | 1/1 | 6.295 |
| Geometry | weak_complementary | 0 | 0 | 197 | 197 | 0/197 | 197/197 | 0.524 |

## Paired interpretation and accuracy

- Noisy pose, equal time: A+B minus A usability is -3 percentage points,
  paired bootstrap 95% interval [-5.5, -1.0]. On the 194 scenes both accept,
  median A+B minus A position error is -0.0465 mm, interval [-0.0636, -0.0291] mm.
  That small conditional accuracy improvement does not erase the six extra
  rejections. Against B, the usability difference is zero, interval [-3.5, 3.5]
  percentage points; common-success median position error improves by 0.1618 mm.
- With equal raw samples, noisy-pose A+B minus A usability is -1.5 percentage
  points, interval [-4.5, 1.5]. Common-success median position difference is
  +0.0386 mm, interval [-0.0030, 0.0591] mm. There is no demonstrated gain over A.
- Complementary geometry, equal time: A+B is complete and usable 200/200;
  both singles are 0/200. Equal-sample A+B is 197/200 versus 0/200 each.
  Equal raw counts retain view-dependent boundary information; this is evidence
  for complementary constraints, not proof that all information budgets are equal.
- Noisy geometry: all three subsets are usable 0/200 under both budgets.
  Equal-time A accepts 14 incomplete estimates, while A+B accepts none.
  Equal-sample A+B accepts one incomplete estimate. None is scored as complete.
- Shared-bias pose, equal time: A+B accepts 195/200; every accepted pose is wrong.
  All 195 joint 95% ellipsoids still cover truth. Position p95 is 10.470 mm.
  Coverage alone does not establish task accuracy or useful uncertainty calibration.

The unchanged prior has 20.494 mm position error and 3 degree orientation error
in every principal condition, outside primary limits. Geometry priors also retain
incorrect nominal dimensions. A prior is a reference, never an accepted observation.

The [full summary](evidence/b2-20261015/confirmation/summary.json) supplies median/p95
position, orientation and observable dimension errors for every subset, budget and
precision/loss setting, exact binomial intervals and paired comparisons.
[Independent corner quantiles](evidence/b2-20261015/confirmation/core-corner-supplement.json)
and [primary metrics](evidence/b2-20261015/confirmation/core-primary-supplement.csv)
supplement the raw corner errors omitted from the core summary. No dimension is scored as
observed when the estimator does not declare it.

A 200/200 usability bootstrap can give a degenerate [1,1] interval; that is not a
population guarantee. Its exact marginal two-sided 95% lower bound is about 98.17%.
Zero wrong among 200 accepted has a one-sided 95% upper wrong rate of about 1.49%,
so the 1% target remains inconclusive. At least 299 accepted independent scenes
with zero wrong are required for that bound. Do not pool different conditions.

## Native ROS protocol and controls

The adapter uses c1702ad and the unchanged production executable. Each request
has a fresh evaluator context/executor and native process. It holds a positive
clock through accepted acknowledgement and exact-UUID COLLECTING feedback, then
runs acquisition/estimation at one times wall time. The entire admission pause
counts in goal latency. Context startup/teardown are exported separately.
Candidate validity is explicitly two seconds, versus the shipped 0.5 seconds.
This measures isolated simulation requests with that profile, not continuous
client reliability or production-default timing.

The owner and independent reviewer each completed thirty controls and sixty
all-condition preflight requests. Each used its own ROS domain. Independent
received-image payload checks, candidate/result field equality, goal UUID,
covariance, acquisition TF, clock and context-lifecycle oracles passed.

The exact codes below are owner-control outcomes.

| Control | Pose B / A+B | Geometry B / A+B |
| --- | --- | --- |
| Exact acquisition TF | DEGRADED_OK / OK | CAMERA_CONFLICT / OK |
| Valid causal 80 ms TF bracket | DEGRADED_OK / OK | CAMERA_CONFLICT / OK |
| Genuine 400 ms moving TF gap | TF_UNAVAILABLE / TF_UNAVAILABLE | TF_UNAVAILABLE / TF_UNAVAILABLE |
| B triplet stale by 800 ms | STALE_OBSERVATION / STALE_OBSERVATION | STALE_OBSERVATION / STALE_OBSERVATION |
| B mask offset by 1 ns | INSUFFICIENT_DATA / DEGRADED_OK | INSUFFICIENT_DATA / CAMERA_CONFLICT |

A-only is unaffected by B's fault: pose accepts with explicit missing B;
geometry returns CAMERA_CONFLICT in this control fixture. Accepted pose A+B
under mask mismatch reports head-only source and missing left, consistent with
minimum-views-one admission. A missing stream is distinct from invalid
participating TF. No rejected control publishes a candidate.
The independent review's geometry mask-skew A+B returned BUFFER_LIMIT, while the
owner's returned CAMERA_CONFLICT. Both are retained no-candidate rejection paths;
unmatched-stream ordering can change which bound or geometry gate terminates first.
The protocol does not require one deterministic code for this one-stream mismatch.

Earlier preflights and reused/unique-name probes remain failed or diagnostic
attempts, outside the population. The corrected fresh-context screening had forty
cancellations and zero failures; its 95% upper failure bound is still about 7.2%.
The [session](session-ros-20261015.md) records these attempts and the separate
unresolved production admission-recovery path. Isolation does not repair that path.


## Native ROS principal pilot

Thirty pilot scenes per condition/mode produce 180 scenes and 540 A/B/A+B requests.
All 540 completed without a protocol failure, and every evaluator context closed.
This uses pilot seeds 100000–100029; it is separate from the fresh 200-scene
numerical study. These populations are not combined for confidence claims.

Each end-to-end usable count has denominator 30 and requires spatial completeness,
all primary accuracy limits, goal send-to-result <=3 s and both ages <=2 s.

| Mode | Condition | Usable A | Usable B | Usable A+B | Accepted A+B | Wrong/accepted A+B | A+B position p95 (mm) | A+B latency p95 (s) | A+B wall age p95 (s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Pose | noise | 29 | 27 | 29 | 29 | 0/29 | 0.737 | 1.055 | 0.501 |
| Pose | shared_bias | 0 | 0 | 0 | 29 | 29/29 | 10.452 | 1.051 | 0.500 |
| Pose | weak_complementary | 0 | 30 | 30 | 30 | 0/30 | 0.032 | 1.051 | 0.500 |
| Geometry | noise | 0 | 0 | 0 | 0 | 0/0 | — | 1.396 | — |
| Geometry | shared_bias | 0 | 0 | 0 | 0 | 0/0 | — | 1.273 | — |
| Geometry | weak_complementary | 0 | 0 | 30 | 30 | 0/30 | 0.333 | 1.444 | 0.893 |

All requests met the 3 s goal-latency budget; the maximum was 1.5154 s.
All accepted results met both age limits: maximum ROS capture-to-issue age
0.9185 s and mapped wall capture-to-result age 0.9209 s (rounded). Thus spatial usability
and end-to-end usability coincide in this pilot. These measurements include
pre-acquisition pause (maximum 4.45 ms), but exclude native startup/discovery and
context construction/teardown. Lifecycle records preserve those outside-goal
intervals. Fixed method order and shared host load prevent a controlled latency
advantage claim.

Noisy pose A+B equals A at 29/30 usable, with a paired difference interval of
[-10, 10] percentage points. Against B's 27/30, the difference is +6.67 points,
interval [0, 16.67]. This small pilot does not establish a general gain.
Shared-bias pose A+B accepts 29/30 and all 29 are wrong, while all 29 joint 95%
ellipsoids cover truth. Complementary geometry A+B is usable 30/30 versus 0/30
for either single camera. Noisy geometry is usable 0/30 for every subset; four
A-only acceptances lack required complete geometry. Observable geometry and
uncertainty remain conditional, and no confidence-calibration claim follows.

The [ROS summary](evidence/b2-20261015/ros-principal/summary.json) retains every
subset's accepted-only pose/orientation/dimension/corner quantiles, coverage,
exact intervals, native codes and paired comparisons. The 30-scene ROS pilot
cannot establish the 1% wrong-acceptance target even in zero-error cells.

## Retained evidence and remaining limits

- [Numerical rows](evidence/b2-20261015/confirmation/requests.jsonl.gz),
  [manifest](evidence/b2-20261015/confirmation/manifest.json),
  [cache index](evidence/b2-20261015/confirmation/cache-index.json).
- [ROS rows](evidence/b2-20261015/ros-principal/requests.jsonl.gz) and
  [manifest](evidence/b2-20261015/ros-principal/manifest.json).
- [Thirty controls](evidence/b2-20261015/ros-controls/summary.json),
  [sixty all-condition preflight requests](evidence/b2-20261015/ros-preflight/summary.json).
- [Diagnostic archive](evidence/b2-20261015/diagnostics.tar.gz) and
  [original-identity index](evidence/b2-20261015/diagnostics-index.json).
- [Original blocked review](evidence/b2-20261015/review/original-blocked-report.md)
  and [protocol remedy review](evidence/b2-20261015/review/remedy-report.md).
- [Final independent report](evidence/b2-20261015/review/final-report.md),
  [38-file review seal](evidence/b2-20261015/review/final-manifest.json) and
  [robotics guidance review](evidence/b2-20261015/robotics-review.md).
- [Evidence layout and integrity instructions](evidence/b2-20261015/README.md).

The complete large pixel cache remains local; compact evidence retains its input
and truth digests. The reviewer audited the actual bytes. Geometry masks are
oracle simulation masks. Real calibration/segmentation distributions, independent
hardware metrology, grasp clearance, continuous-client operation and the
production admission-recovery remedy remain unverified. The production estimator
and installed defaults remain frozen at 434803c.

**B2 is closed within the approved simulation benchmark scope.** Independent
technical verification of populations, statistics, documentation and compact
evidence passed; see the final report above. The original blocked and remedy
reports retain their historical dispositions.
Editorial approval and ASD-STE100 language review remain pending separately.
