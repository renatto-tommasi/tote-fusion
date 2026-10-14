# Independent M3 review — 2026-10-14

Status: accepted for the specified M3 simulation candidate handoff. No open blocking findings. R1 is resolved; hardware, production calibration and model-manager acceptance remain outside this milestone.

## Scope and authority

This reviewer started without the implementation conversation, read AGENTS.md,
the canonical robust-minimal-code skill, TOTE_FUSION_NODE_SPEC.md, design decisions
and milestone/runbook documents, then inspected the C++ core, geometry extraction,
ROS adapter, interface, fixtures, evaluator and evidence tooling. Phase 2 and
fresh-context milestone review were explicitly authorized. Production source was
read-only during this review. Evidence is in `verification/review_m3`; all final ROS
checks use domain 90. No robot control or planner/model commit was performed.

## Findings

No blocking production defect was reproduced. One evidence issue was found and corrected:

**R1 — joint numerical fixtures use nominal orthogonal wall placement.**
The original `geometry_fixture.hpp::boundaries` placed width endpoints on x=-0.300 m and length
endpoints on y=-0.200 m regardless of the true other dimension. In the 620/420 mm
population, surface fixtures instead put the corresponding outer walls at
x=-0.310 m and y=-0.210 m. Both endpoint sets are therefore 10 mm off those
physical wall planes. Their lateral separation is correct, which is exactly
what the boundary residual measures, so this does not establish a production
failure. It does limit the claim that the joint sensitivity/covariance population
uses physically consistent endpoints. The independent pixel renderer uses true
extents for both axes and is not affected. The owner corrected the fixture to receive complete physical dimensions. The
reviewer independently constructed endpoints without calling that corrected
helper; all 300 old-seed rows match the owner's corrected `noise-physical.csv`
exactly except wall timing. R1 is resolved, with the inconsistent historical
population preserved. Only three test/evaluation files changed; runtime core,
geometry, ROS node and pixel renderer are unchanged.

## Verified behavior

Source inspection establishes one shared bounded pose/joint solver, immutable
model/configuration ownership, acquisition-time TF with bounded interpolation,
explicit geometry mode, and publication only after final gates. Geometry input
requires positive finite background, local planar support, paired coplanar
vertical endpoints and independent fitting/held-out support. Surface and boundary
camera consistency gates prevent low-noise/dense observations from concealing a
received conflicting camera. The final endpoint-side gate is present.

The data-only full joint information matrix, with documented state scaling,
establishes rank independently of the prior. The complete covariance retains
pose/dimension cross terms and maps left tangent translation to ROS position.
Systematic floors are explicit; confidence is marked uncalibrated. Pi-yaw
ambiguity is conditional on a named equivalence profile or rejects. A measurable
width alone does not establish a fully observable pose candidate.

Candidate schema 3 carries the unchanged parent revision/hash, proposed generator
and geometry hash, explicit estimated dimensions, uncertainty, unresolved axes,
conditional inner opening and local collision envelope. Unobserved axes do not
shrink. No source dependency or subscribed runtime interface accesses evaluator
or simulator truth; truth appears only in test/evaluation artifacts.

Independent copied-binary runs passed 46 core, 14 head action, 13 multiview action
and 7 geometry action cases: 80 individual test cases with zero failures.
The final head evidence is `head-action-domain90.xml`; its initially randomized
harness domain was explicitly replaced with 90 and the full suite rerun. The
owner's colcon total of 84 includes four CTest wrapper records. ROS evidence
checks capture-time motion, provenance, missing/conflicting cameras, unsupported
mode/revision, queue limits, reset/cancellation, and zero candidate publication
on rejection. XML, logs, copied binaries and all 26 source-file hashes are kept
in the review bundle.

The source-current 300-row numerical replay has 100 prior rows, 100 equal-weight
accepted rows and 100 robust accepted rows. Its values exactly match historical
`verification/m3/noise.csv` except wall timing. This verifies that the final gate
does not change that qualified-boundary population; it does not turn those
repeated seeds into new validation seeds or pixel extraction evidence.

## Evidence provenance and limits

The initial `verification/m3/corpus` is a historical source-pinned experiment.
Its saved core and numerical regression source differ from the reviewed source;
it predates the endpoint-side camera residual gate. It is not final-source
runtime/performance evidence. The refreshed `corpus-reviewed` matches all reviewed runtime source hashes.
Its post-run snapshot also captures the three subsequent test-only fixture
corrections. The four-mutant bundle matches all 22 estimator source hashes at
the time of mutation, and each named failure was independently checked in XML.
The restored suite passes; its runtime source is unchanged by the fixture fix.

The simulation has explicit test-only masks, frontal views and an object stationary
relative to the body frame. It does not validate image-derived M3 segmentation,
actuated Gazebo wrists, hardware calibration, unrestricted localization or hidden
physical geometry. Shared rigid calibration can bias pose while preserving size;
common scale error is indistinguishable from a size change. Width/length accuracy
is conditional on accepted observations and the nominal unresolved model.

The final-source pixel corpus independently audited 125 requests: 60/60 clean
accepted, 21/60 noisy accepted (10/30 two-camera, 11/30 three-camera), and exact
GEOMETRY_CONFLICT, INVALID_INTRINSICS, INSUFFICIENT_GEOMETRY, CANCELED and
SIM_TIME_RESET outcomes for the five separate negatives with empty candidates.
Final p50/p95/p99/max action wall latency is 2.151/2.252/2.275/2.280 seconds
with two cameras and 2.542/2.713/2.735/2.740 seconds with three. Maximum accepted
width/length error is 0.576/0.278 mm across the workload. This high rejection rate must remain visible, and failed cases carry
no accuracy claim. Fixed simulation-clock action wall latency is a shared-host
measurement, not a real-time or continuously advancing-clock guarantee. Three
sigmas plus nominal padding do not make a certified world-pose collision model.

## Reproduction

The standalone source build succeeded; its log retains four compiler warnings
inside Eigen headers, exposed by the direct compiler include flags.

The reviewer also compiled the pinned production core and geometry source
directly, with assertions active, and evaluated independently constructed
physical endpoints on unused seeds 5000–5099. All 100 robust and 100 equal-weight
trials accepted. In the 50 noisy robust trials, p95 width/length errors are
1.097/1.472 mm (maxima 1.569/1.860 mm); p95 pose translation error is 6.341 mm
(maximum 7.303 mm). All dimension errors fall within 1.96 reported sigmas in
this finite population. Source-build and checked-library results match exactly
except wall timing. This remains a qualified-boundary sensitivity check, not
pixel validation or general uncertainty calibration.

`verification/review_m3/reproduce.bash` reruns the copied-binary numerical and
domain-90 ROS checks and independently compiles the pinned source with physical
endpoints for old and unused noise seeds.
The manifest and provenance-check JSON identify exactly which source and binary
bytes were inspected and distinguish the historical initial corpus.

## Final disposition

Accept M3 within its explicitly authorized simulation scope: identifiable width
and length proposals, honest local uncertainty, conditional collision bounds,
independent failure gates and external model ownership. The evidence fixture
finding is fixed and independently reproduced. No production correction was
required by this review. The useful next change, if separately authorized, is
uncertainty-aware pixel boundary extraction evaluated on independent images and
seeds, retaining the present rejection/publication gates. Production calibration,
automatic model acceptance and robot control are not implied by this approval.
