# Implemented milestones and review history

This is the standalone export of the completed simulation contribution. The
original workspace contains the large recordings, sealed manifests, recordings
of operation and simulator/robot dependencies. This repository carries the two
contribution packages, portable tests, selected evaluation summaries, recorded
point exports, and historical reviews. Original absolute paths in historical
documents identify evidence provenance; they are not standalone instructions.

## M1 — head-camera pose

Implemented a ROS-independent C++ geometry/registration core, a bounded ROS
action adapter, strict matched depth/mask/CameraInfo validation, acquisition-time
TF, immutable nominal model snapshots, explicit pi-yaw policy, held-out fit and
observability checks, candidate expiry and bounded cancellation/failure.
Simulator truth remains external to estimator dependencies. Nominal geometry
is fixed in this mode.

Lessons: low residual does not establish full pose observability; a strong
prior cannot replace data information; latest TF is not acquisition-time TF.
Keep object/model identity and local symmetry assumptions explicit.

## M2 — multi-camera pose

Implemented one-to-three camera configuration, distinct optical-frame/topic
contracts, per-camera noise, joint fitting, temporal and camera-held-out checks,
camera-consistency rejection, missing-camera provenance and `DEGRADED_OK` when
the requested minimum still supports acceptance. Workload, retained memory and
steady wall deadlines remain bounded.

The independent M1/M2 review exposed TF interpolation-support/topology issues
and truth-derived mask coupling in a test fixture. Corrections added explicit
TF history/support contracts and separated simulator truth from registration
inputs. Review notes distinguish corrected findings from remaining limits.

Lessons: interpolated TF must have bounded support, not just a lookup stamp;
calibration/topology changes invalidate a request; whole-camera holdout catches
some conflicts that pooled residuals obscure. Synthetic masking is test-only,
not demonstrated production segmentation.

## M3 — observable geometry refinement

Implemented width/length proposals from supported physical boundaries,
independent fitting/held-out camera boundary gates, scaled data-only joint
pose/dimension information, joint covariance, explicit unresolved dimensions,
conditional inner opening/collision geometry and validated immutable model
revision updates through a separate workspace consumer prototype. That model
manager is outside the two published contribution packages; commits require
external integration. Pose-only requests remain
separate from geometry profiles. The node publishes proposals only.

Final realistic-resolution action evidence: 60/60 clean accepted, 21/60 noisy
accepted, 39/60 noisy rejected. Four M3 mutation checks detected injected bugs.
The accepted noisy screenshot is seed 3093; rejected seed 3090 lacks independent
fitting support for a held-out camera dimension. Original 84 reported checks
include individual cases and wrappers; standalone rerun evidence is recorded
separately in this repository.

An independent M3 review found no remaining production-code blocker within the
simulation contract. A reporting helper silently substituted zero for a missing
accepted-error field; a subsequent fix uses explicit accepted-only indexing,
fails loudly on missing evidence and reproduces the valid evaluation. The
runtime estimator was unchanged by that reporting correction.

Lessons: crop edges are not physical object boundaries; a single wall/prior
cannot justify unsupported dimensions; height/thickness/underside remain
unresolved; do not report fabricated zero errors for rejected or missing data.
Rejecting noisy data is a valid outcome, not a reason to weaken gates.

## Public package preparation

The two contribution packages were exported and built from a clean ROS Jazzy
underlay without the simulator workspace overlay. All 84 reported checks passed.
The README includes actual recorded-cloud RViz comparisons, input/configuration
and output/action remapping, explicit external recovery guidance and a robotics
review. Package descriptions/contact metadata were refreshed; estimator runtime
code, interfaces and configuration are unchanged.

Lesson: document the shipped package boundary. The original workspace model
manager prototype is an external consumer, not a library shipped here.

## Evidence and limits

- [M1/M2 independent review](review_m1_m2.md).
- [M3 independent review](review_m3.md).
- [M2 evaluation](../evidence/m2-evaluation.json).
- [M3 evaluation](../evidence/m3-evaluation.json).
- [M3 mutation summary](../evidence/mutations.json).
- [Independent review machine-readable report](../evidence/independent-review.json).
- [Original final test summary](../evidence/tests.log).
- [Standalone publication tests](../evidence/standalone-tests.log).
- [Detailed lessons](lessons_learned.md).

Covariance is conditional and uncalibrated. This contribution does not establish
hardware accuracy, production segmentation, production timing, globally unique
yaw, arbitrary geometry recovery or certified collision safety. Base-motion
recovery and model commit are external integrations.
