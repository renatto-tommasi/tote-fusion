# Decisions for M1 implementation

Prepared 2026-10-14 against [specification v0.1](../TOTE_FUSION_NODE_SPEC.md).
These decisions cover the simulation exercise. The first C++ core/action and
versioned interfaces are now implemented; see the [standalone usage](../README.md#launch-and-request-an-estimate).
Final M1 evidence and acceptance scope are recorded in [milestone notes](milestones.md#m1--head-camera-pose).

## Contract and scope

Implement one bounded `EstimateTote` action server with a pure C++17/Eigen core
and thin ROS adapters. Enable the fixed head camera first. Freeze the model
snapshot per request; reject overlapping goals, stale revisions and geometry
mode. Publish a versioned pose candidate and diagnostics only after validation.
Cancellation, reset, overload and rejection publish no candidate. No component
moves the robot or accepts a planning-model revision.

Use `ament_cmake`, `rclcpp`, `rclcpp_action`, `tf2_ros`, `tf2_eigen`, standard
sensor/geometry/diagnostic messages, `rosidl_default_generators`, Eigen and
yaml-cpp. Test with `ament_cmake_gtest` and `launch_testing_ament_cmake`/Python
`launch_testing`. No PCL, GPU, database or new controller is needed for this
first implementation. M0's compile/link probe checks these installed libraries;
it is not an estimator test.

## Input and frame decisions

The measured simulation depth is registered axial optical Z, `32FC1`, metres,
640 x 480. Use the corrected live CameraInfo, never a hardcoded focal length.
Do not apply the Gazebo principal-point/cloud correction a second time.
Support `16UC1` millimetres only with a distinct validated test fixture in M1.
Require matching mask/depth/CameraInfo frame, size and acquisition stamp; masks
are independently produced from registered RGB/depth, `mono8`, 0/background
and 255/tote. The current blue-object baseline can select other blue objects.

Use `base_link` for estimator inputs, registration and candidates. Lookup
`T_base_link_camera` at acquisition time; never substitute latest TF.
Use the geometry's bottom-centre origin as `tote`, an identity alias of the
asset's `tote_base`; never publish physical-truth `tote_base` TF.
See [frames and conversion](frames.md).

The existing planning JSON uses `world`, `attempt_id` and `geometry_sha256`.
It is an environment prototype, not the specified versioned M1 model contract.
M0 prepares a separate immutable `tote_model.yaml` using only that public bag
and robot static TF, with explicit local revision 0 and a hash-verified copy of
the nominal geometry. Resolve its geometry filename relative to the snapshot.
The YAML's prior covariance is a proposed **uncalibrated regularizer**:
50 mm translational and 10 degree rotational standard deviations in a left
body-frame tangent. It is neither measured uncertainty nor an accuracy gate.
M1 must validate the schema, finite/unit quaternion, geometry hash and covariance
and freeze them per goal. The original bag/model is preserved unchanged.

The existing evaluator accepts `PoseStamped` in `world` on
`/tote_estimation/pose`. During M1, add a test-only candidate adapter that checks
candidate identity/revision/time and converts `base_link` to `world` at the
candidate capture stamp. Keep that adapter outside the estimator's dependency
graph. Extend evaluator outputs for held-out trials and symmetry-aware metrics.
Do not rename or weaken the estimator contract to match this prototype.

## Registration decisions, before implementation

1. Construct immutable bounded surface patches from the nominal CAD: inner and
   outer walls, inner floor, narrow rim strips and stacking-foot faces. Existing
   YAML lists wall/floor planes, rim edges and collision boxes; M1 must turn
   them into bounded queries. An infinite plane is not an adequate model.
   Exclude handle apertures, the documented label band and bevel neighbourhoods.
   Do not rescale the nominal CAD to its label-expanded visual bounds.
2. Keep camera origin/ray and frame/time/calibration provenance with every point.
   Reject patches that face away from the camera or are occluded by another
   modeled surface along that ray. Closest Euclidean surface alone is inadequate
   for thin inner/outer walls. Recompute correspondences as the pose changes.
3. Use signed point-to-plane residuals on the valid bounded patch, with edge/rim
   support evaluated separately. Weight by declared camera/depth noise and
   spatial/time sampling; normalize camera groups so pixel density is not a
   confidence multiplier. Use Huber loss and a damped Gauss-Newton solver with
   Eigen. Use left updates `T <- Exp(delta_xi) T`; bound iterations and report
   total solver iterations, final support/residuals and convergence rejection.
4. Start from the frozen prior and independently fit the rectangular tote's
   supported pi-yaw counterpart. M1 supports the tested local prior basin;
   unrestricted edge-based relocalization is outside this fixed-head release.
   Reject nonfinite steps, inadequate support and nonconvergence. Bound maximum
   iterations and wall-time; cancellation must be checked between iterations.
5. Use time-separated frames for fitting and hold-out, with no sample overlap.
   Adjacent pixels or repeated noiseless frames do not become independent views.
   Reserve additional independently perturbed trials for covariance validation.
   Compare unchanged prior, unweighted fit and robust weighted fit on identical
   held-out inputs; truth is accessed only by the evaluator.
6. Compute data-only information rank/conditioning without the prior. A strong
   prior cannot create observed DOFs. Report weak modes and coverage separately
   from residual/convergence. Gate training and hold-out fit, support, data age,
   model identity, visibility, observability and ambiguity before publication.

Solver tolerances, robust-loss scale, noise model, rank cutoff, capture/point/
memory bounds and candidate expiration must be explicit checked configuration
in M1, with units and a named simulation profile. They have not been measured
or accepted as production thresholds. Select and document exercise values in
the M1 design/configuration before running acceptance experiments; this prep
does not invent an accuracy pass/fail tolerance.

The first implementation's explicit exercise values are listed in the
[configuration instructions](../README.md#input-topics-and-calibration). Data weight is
capped at 100 effective points per acquisition frame. The initial one-point
frame normalization excessively strengthened the prior; nondegenerate pose
offset tests exposed that bias. The effective-count assumption and covariance
floors are simulation assumptions. Calibration seeds 1000..1059 required a
95% ellipsoid scale of 1.2034; the release rounds it upward to 1.25 and evaluates
separate seeds. This is population-specific; the published calibration flag
remains false for general confidence. See the evaluation for coverage and
nonconvergence counts.

## Covariance and symmetry

Use `delta_xi = [rho_x,rho_y,rho_z,phi_x,phi_y,phi_z]` with
`T' = Exp(delta_xi) T`, expressed in `base_link`. For ROS pose ordering, map
the tangent covariance with `A = [[I, -skew(t)], [0, I]]`, since
`delta_position = rho - skew(t) phi`. Test this conversion with nonzero
translation/rotation. Inverse Hessian is a local sensitivity estimate; account
for frame correlations and assess empirical coverage on repeated held-out noise
trials before claiming calibrated confidence.

The nominal rectangular tote, with labels excluded, has pi-yaw symmetry.
The prior may select a local basin but does not establish a unique global yaw.
Default to `AMBIGUOUS_SYMMETRY` rejection unless an explicitly named simulation
profile permits a pose equivalence class. Such a profile must expose the
alternative hypotheses, report conditional local covariance, and evaluate
orientation modulo the modeled symmetry. Never publish an unconditional
precise 6-DOF claim from this scene alone. A planar-patch fixture must fail the
data-only rank check even with a precise prior.

## Assumptions, risks and decisions requiring real-robot confirmation

The stationary official TIAGo geometry plus local camera overlay is sufficient
for this simulation M1. Head sensors, axial-depth conventions and static robot
TF are checked for the captured configuration. Full PAL actuation, wrist
calibration/moving TF, hardware behavior and production timing are unverified.
The generic EU6423 is not certified hardware CAD. Its visual labels extend the
0.600 x 0.400 m footprint by 1 mm per side; the nominal height is 0.230 m.
Sharp collision boxes approximate the source's 1.2 mm bevels.

Real topics/calibration, model-manager ownership/interface, authoritative
revision numbering, production prior/noise models, candidate expiry, accuracy,
grasp clearance and p95/p99 deadlines require later confirmation. None blocks
the stated stationary simulation exercise. M2 and Phase 2 remain separate
work; Phase 2 requires explicit approval.

No top-level Git repository exists. Record that fact instead of inventing a
commit/branch. Source snapshots, hashes, pinned upstream commits and retained
vendor patches provide local provenance. Repository publication and production
deployment are outside this preparation.

## M2 implementation start — authorized 2026-10-14

Keep the sealed M1 bundle unchanged. Extend the same C++ core/action; do not
implement dimensions, model commits, controllers or physical robot motion.
The camera catalog is an immutable YAML snapshot of the checked head/left/right
interfaces. Enable head+left first, then head+left+right with explicit profiles.
Camera topics, optical frames and axial conventions retain their existing
backend corrections. All three cameras require fresh known-target preflight.

Key queues/deduplication by `(camera, capture_stamp)`, preserve calibration per
camera, and retain each TF lookup's camera/stamp. No timed/latest TF substitute.
Use time blocks shared across cameras for M2 fit/hold partitioning (100 ms
exercise blocks), so identical acquisition times cannot cross the split.
M1 keeps its existing alternating head-frame partition and information weights.
For M2 normalize each camera's information across its frame/pixel count and use
its declared sigma; repeated copies cannot make a dense camera dominate.
The M1 empirical covariance scale is not a calibration of the M2 population.

Missing cameras may produce `DEGRADED_OK` only if the goal's minimum distinct
cameras is satisfied and remaining data passes all quality/rank gates. List
missing cameras explicitly. Invalid received data/calibration/TF still rejects;
missing data never becomes guessed data. Reject view-specific signed plane
bias above the declared 3 mm exercise gate before publishing an averaged pose.
This detects coherent translational disagreement on supported surface normals;
it does not identify an invisible shared calibration bias or guarantee detection
of every rotational conflict. Per-camera held-out support/residuals are gated.

Exercise capture budgets: head 40 frames/20,000 points; multi-view 120 total
frames/60,000 points, 64 MiB logical queued payload, 2 s maximum capture,
10 s steady wall deadline, 2 s candidate validity in integration trials.
All sigmas initially 3 mm; the catalog and tests permit independently specified
camera noise. Moving-wrist proof starts with nonidentity, time-varying ROS TF
fixtures; simulated joint-to-TF interpolation and newest-TF mutations must be
verified. Static Gazebo wrist views do not constitute a moving-robot proof.
Record tests, deliberate mutation failures, static/live outcomes and outstanding
M2 uncertainty/performance/physical-motion limits in a separate handoff.

## Completion authorization and robotics review — 2026-10-14

The user explicitly authorized completion through the remaining milestones,
fresh-context reviews after milestones, and a separate robotics expert for
design decisions. This supersedes the earlier M2/Phase 2 authorization gate;
it does not authorize robot control or automatic planner/model commits.

The robotics expert selected independent analytic moving RGB-D/joint replay
alongside the existing image-derived Gazebo integrations for M2. The replay
records real serialized sensor/clock/joint/TF messages with acquisition-bracketing
transforms and separately rendered midpoint images. Its masks are declared
test fixtures. No claim of actuated Gazebo wrists or hardware motion is made.
Measure 30 clean and 30 noisy requests for each active two-/three-camera workload,
retain rejections, and describe the workload and finite-sample quantiles.

Use separate seeded camera-specific and shared calibration perturbations for
M2 covariance assessment. Retain prior/equal-weight/robust comparisons and
calibration/validation splits; retain the hardware calibration flag as false.
The exact population and observed statistics belong in the M2 evaluation.

For M3 the expert selected width first, then length through one paired-boundary
constraint implementation. Height and wall thickness remain unresolved without
views that establish the necessary physical boundaries. Require positive
background/free-space evidence at both endpoints; missing/cropped/occluded
points are not physical edges. Full candidate acceptance requires data-only
joint pose/geometry rank, even when a parameter-only width measurement is valid.
Adopt a named 2 mm systematic dimension floor and three-sigma disagreement gate.
Propose conservative uncommitted collision bounds without shrinking unseen axes.
The specified project ends at reviewed M3; production calibration, model-manager
transactions and robot control are downstream work.

## M3 implementation contract — robotics decisions before coding

Geometry mode is separately configured and requested. Run fixed-geometry pose
optimization first and retain boundary mismatch, then the declared joint solver;
do not catch an arbitrary pose-only failure and retry with geometry. Width is
axis Y from an X-normal wall, length axis X from a Y-normal wall. Require both
silhouette endpoints, positive finite background at least 30 mm behind the
object, local planar support (3 mm RMS, normal within 10 degrees, 60 mm prior
association basin), and paired vertical lines (12 samples each, 50 mm span,
3 mm RMS, within 10 degrees of vertical) on the same wall. Image borders,
mask-only crops, invalid depth and nearer occlusion do not establish an edge.
Opposing line separation is independently measured per camera in fitting and
held-out frames. Use a 2 mm systematic dimension floor and a three-sigma
cross-camera disagreement gate, without averaging an unexplained conflict.

Use the existing robust SE(3) solver with appended supported dimensions, not a
second registration implementation. Boundary residual is signed lateral
coordinate minus half dimension. Surface correspondences are regenerated with
current parametric boxes. Joint data-only rank uses scales 10 mm translation,
0.05 rad rotation and 10 mm dimensions, with ratio 1e-5. Priors cannot establish
observability. Retain complete pose/dimension covariance and cross terms;
confidence remains uncalibrated and conditional on the pi-yaw class and nominal
unresolved parameters. Geometry bounds are +/-60 mm from nominal, an exercise
association assumption.

EU6423 generator v1 regenerates the immutable 17 boxes. In X/Y, thin 2.5 mm
intervals translate by signed half-delta; intervals crossing zero expand both
ends; handle-side intervals move only the outer bound, keeping +/-55 mm inner
edges. Check supported interval classes and positive sizes; never uniformly
scale wall thickness or handle openings. Z features remain unchanged.

Candidate schema carries only estimated parameter names/values/sigmas, boundary
support by camera and axis, joint state order/covariance, generator version and
proposal hash, conditional inner opening, unresolved height/thickness/hidden
regions, and collision lower/upper bounds. Horizontal envelope uses
max(nominal, estimate+3*sigma) plus 2 mm per side; unobserved axes retain nominal
extents. Z bounds extend 2 mm below zero and above nominal height. This is
conditional conservativeness, not a hardware or world-pose safety guarantee.
Common calibration scale error cannot be distinguished from size by these
inputs; retain a limitation test and do not claim online calibration.

The first frontal ROS positive exposed bounded fixed-stage nonconvergence:
wrong 600/400 mm dimensions cannot fit opposing 620/420 mm surfaces, and
correspondences can oscillate. The robotics reviewer refined the contract:
fixed-stage convergence is diagnostic in geometry mode; return it explicitly
with iterations and held-out surface/boundary residuals, then always execute
the requested joint stage. Pose-only fixed-stage and every joint-stage
nonconvergence reject. This is a declared two-stage algorithm, not exception
recovery. Nonfinite/out-of-basin steps, invalid factorization, inadequate support,
cancellation/reset/deadline faults abort either stage. Both stages and both
symmetry hypotheses count toward the bounded workload (at most 160 iterations).

The robotics reviewer assessed the final pixel corpus: 60/60 clean accepted,
21/60 noisy accepted, with explicit insufficient-support/fit/conflict rejections.
The specification sets no minimum noisy acceptance rate. Proceed with review
under the simulation candidate scope, prominently retain the high rejection
rate and conditional accuracy, and do not relax gates to improve the count.
Future extraction work should use uncertainty-aware support tests and independent
validation seeds. Qualified-boundary uncertainty trials remain distinct from
pixel extraction validation; fixed simulation-clock wall latency is not a
real-time or hardware guarantee.

A reproduced wall-parallel camera-bias case showed that correct line separation
and global boundary RMSE can hide inconsistent endpoint placement with unequal
noise/density. The robotics expert required a 5 mm residual gate per participating
camera, dimension axis and endpoint side in both fitting and held-out data.
Retain global/rank/dimension-disagreement gates. Conflicting received data rejects
with CAMERA_CONFLICT and cannot disappear. The original 12 mm biased fixture
accepted at 2.596 mm global boundary RMSE; the restored gate must reject it,
while valid unequal density/sigma remains accepted.
