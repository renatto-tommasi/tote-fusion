# Paired fusion benchmark contract v1 — 2026-10-15

Starting implementation: `434803c4bc30c4d8f759dc8409f13e7bfdfce063`.
Authority: user requested cached A/B/A+B/prior comparisons and delegated
provisional evaluation thresholds to robotics experts. No hardware operation,
model commit or production registration change is part of this benchmark.
Advisors: `/root/robotics_decisions` and fresh-context `/root/benchmark_advisor`.
Both agreed on the primary profile below before results were generated.

## Requirements and stages

- B01: Generate both cameras once per independent scene. Cache depth, masks,
  intrinsics, acquisition-time transforms and model prior. Keep truth separately.
  Every subset and ablation must identify the same cache digest.
- B02: A-only, B-only and A+B retain original camera indices 0/1 and TWO configured
  sigma entries. This freezes the existing multi-camera normalization and 1.35
  pose covariance multiplier. This is a common-configuration subset experiment,
  not the separately calibrated one-camera M1 deployment profile.
- B03: Compare equal capture duration first, then equal surface and boundary
  sample budgets independently. Deterministic selection uses observed IDs only,
  without truth, acceptance-based regeneration or replenishment. Preserve
  shortages, invalid inputs and observability rejection. The equal-sample surface
  budget is at most300 per partition (below the existing per-frame cap); boundary
  budgets use the smaller nonempty observed count. Report actual and capped counts.
  Equal raw sample count does not imply equal effective information.
- B04: Cross declared-noise/equal precision with Huber/squared loss. Equal precision
  changes only quadratic denominators in an isolated generated source copy;
  per-camera/boundary sigmas, Huber transitions, quality gates and priors stay
  fixed. Production source/defaults remain unchanged. Record instrumented hashes.
- B05: Pose trials use correct nominal dimensions; geometry trials vary width and
  length. Independent finite-box first-hit rendering supplies sensor inputs.
  Production projection and pixel boundary extraction supply numerical factors.
  No estimator raycast creates input observations. Test masks remain oracle masks.
- B06: Export every request, including rejection, with a stable scene ID, subset,
  budget, factor settings, code, selected counts, timing and full accepted state/
  covariance. Rejected estimates have null accuracy/confidence, never zero errors.
  Prior is a reference, not an accepted sensor estimate. Convert its tangent pose
  covariance correctly; do not invent dimension uncertainty.
- B07: Report acceptance, accepted-only median/p95 accuracy, useful/all, wrong/all
  and wrong/accepted. Evaluate only declared dimensions, report missing required
  dimensions separately, and retain full joint covariance/cross terms. Score yaw
  modulo pi symmetry. Compare joint-success error on paired scene intersections
  only as a secondary selection-dependent metric. Use scene-level paired bootstrap
  intervals and exact binomial confidence limits. Do not pool conditions/methods.
- B08: Pixel-core timing includes extraction and fitting, both symmetry hypotheses
  and geometry stages. It does not measure ROS acquisition-to-publication age,
  TF admission, dropout policy or action latency. Those require separately
  identified same-byte ROS replay. Missing age is null, not guessed success.
- B09: Pilot 30–50 scenes/condition. Freeze configuration before fresh principal
  populations of 200 scenes/condition. Pilot seeds are excluded from confirmation.
- B10: Independent fresh-context verifier must reproduce checks before each
  benchmark-stage closure. No superiority/hardware-confidence claim from the
  previous 21/60 acceptance result, or from this harness merely existing.

## Preregistered provisional simulation criteria

These are stationary-picking geometric proxies, not known robot/gripper clearance
requirements. Actual insertion/grasp clearances, calibrated camera distributions
and task deadline ownership remain unresolved.

Primary: centre error <=5 mm; pi-modulo orientation <=1 degree; each REQUIRED
width/length error <=3 mm; maximum corresponding outer-envelope corner error
<=15 mm. Missing required dimensions mean incomplete, not zero error or a wrong
measurement. Accepted-but-wrong means a declared spatial estimate violates a
primary spatial limit. Report incompleteness and timing failures separately.

Timing proxy: goal-to-result <=3 s; latest-capture age <=2 s. ROS replay must
measure age in acquisition clock and its steady-wall replay mapping. Pixel-core
has no measured goal/action age and cannot establish end-to-end usability.
Secondary sensitivity: 10 mm / 2 degrees / 5 mm, with the SAME corner/timing
limits. Never replace primary failures with secondary successes.

Accepted-but-wrong target <=1% among accepted independent scenes, judged by a
one-sided exact 95% binomial upper bound. Zero wrong among 200 accepted gives
about 1.49%, so that experiment alone cannot prove the target. At least 299
accepted independent scenes with zero wrong are needed for an upper bound below
1%. Report inconclusive rather than silently relaxing the target.

The expert initially suggested a shorter timing profile, then withdrew it after
reviewing the stationary workload and published ~2.5 s geometry latency. This
decision predates benchmark results. Existing 0.5 s default expiry is not a
validated task deadline. All thresholds are recorded in `criteria.json`.
