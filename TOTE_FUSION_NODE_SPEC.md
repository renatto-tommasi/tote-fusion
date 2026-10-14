# Tote Fusion Node — Engineering Specification and Coding-Agent Handoff

**Document version:** 0.1 — proposed implementation contract  
**Date:** 2026-10-12  
**Status:** Implemented through reviewed M3 in the authorized simulation scope; environmental and acceptance thresholds remain assumptions, not verified hardware requirements.  
**Implementation checkpoint (2026-10-14):** M1 and M2 are finalized and independently reviewed in simulation; see [M2 evaluation](evidence/m2-evaluation.json) and [review](docs/review_m1_m2.md). M3 is also finalized and independently reviewed; see [milestone notes](docs/milestones.md). The requirements and later-milestone gates below remain in force.
**Scope:** One ROS 2 model-based multi-view *tote estimator* (plus minimal ROS interfaces and an independent test evaluator). **Not** grasp planning, robot motion, pick/place execution, task orchestration, or automatic acceptance of planner model changes.

## 0. Instructions to the coding agent

1. **Read this entire specification before writing code.** Summarize the intended contract, assumptions, risks and unresolved decisions. Inspect the target workspace and any repository-specific `AGENTS.md` / contributor guidelines before editing. Do not assume an existing target repository was authorized just because its name appears here.
2. **Implement Phase 1 only** unless explicitly asked to advance to Phase 2. Build the smallest vertical slice (one stationary head/body camera, synthetic open tote, fixed dimensions), then add multi-view support in separate testable increments.
3. Prefer a **maintainable C++17 ROS 2 `ament_cmake` package**, a small pure-geometry/estimation library, thin ROS adapters, explicit configuration and deterministic tests. Use existing standard ROS messages where they accurately capture semantics; introduce custom messages only for the candidate estimate and bounded request/result.
4. Preserve the boundary between **simulator physical truth**, **image-derived sensor observations**, **current planner model** and **proposed model revision**. Runtime estimator code must not query simulator truth poses, dimensions, instance IDs or ground-truth segmentation labels. Only the separate evaluator sees truth.
5. Do not silently select success thresholds, invent robot calibration, claim physical safety or alter production robot/controller packages. If a required interface or library differs in the installed ROS release, report the discrepancy and propose the smallest adjustment.
6. **Demonstrate that tests detect defects:** for key tests, deliberately disable/reverse the relevant behavior (wrong transform, stale-data bypass, ignored geometry discrepancy, etc.), show a meaningful test failure, then restore correct behavior.
7. At completion provide: file-by-file change summary; commands and pinned environment; test results and logs; failed/skipped checks; example input/output; unresolved limitations; next milestone. Never claim something was tested if it was only specified.

## 1. Source of this specification and authority

### User-established design

The user's handwritten note *“model-based, multi-view estimator”* (three pages, 2026-10-12) defines: three RGB-D views (right wrist, left wrist, head); camera-specific semantic processing; transformation into a common body frame while preserving origin; alignment with a CAD model; Phase 1 state `T_B_T ∈ SE(3)`; Phase 2 state `{T_B_T, θ}`; bottom-center tote frame; and a desired model/collision representation as output. The last output is refined here into a **candidate**, not an automatically committed collision-model update.

The separate *Tote model alignment — staged simulation workbook* (11 pages, 2026-10-12) establishes: Gazebo first, Isaac Sim later; staged fixed-camera to wrist-camera experiments; independent physical and planner models; no simulator-truth leakage; explicit frame/time convention; observability, uncertainty, fault injection and a validated revision boundary. Its parameters `+20 mm` translation and `+3°` yaw are **illustrative test perturbations, not acceptance criteria**.

### Relevant reviewed knowledge-base notes

Knowledge repository: private `renatto-tommasi/renattos-knowledge`, `main` observed at commit `9031b9ef2b6cb6701eaf06722bb6cd27822c9497` when drafting this document. Re-read current versions if the repository advances; do not treat cited notes as permission to edit the vault.

| Source path | Engineering obligation derived from it |
| --- | --- |
| `Concepts/Maintainability.md` | Operability (logs, diagnostics and runbooks), simplicity (clear ownership), evolvability (versioned contracts and tests). |
| `Concepts/Reliability.md` | Explicit service boundary, contained sensor faults, observable degraded/rejected behavior, safe restart assumptions. |
| `Concepts/Performance.md` | State operation, camera workload and timing boundaries; measure p50/p95/p99, drops and error rates. |
| `Concepts/Scalability.md` | Measure frame rate, cameras, point count and queue growth before parallelizing/distributing nodes. |
| `Concepts/Tests must distinguish behavior.md` | Nondegenerate test inputs; verified failure under representative deliberate mutations. |
| `Sources/2026-10-02 Just et al - Mutation testing and robotics.md` | Fault injection and expected-outcome bag regression as complements to code mutation; bag replay is not a closed-loop proof. |
| `Sources/2026-10-07 DDIA - Trade-Offs in Data Systems Architecture.md` | Keep authoritative model state separate from derived estimates; define how derived state is refreshed and versioned. |
| `AGENTS.md` | Trace provenance and uncertainty, read narrowly, do not change the knowledge vault without authorization. |

Public starting tote asset: [`EU6423.glb`](https://github.com/danielrosehill/storage-box-3d-models/blob/main/models/EU6423/EU6423.glb), [`units.json`](https://github.com/danielrosehill/storage-box-3d-models/blob/main/specs/units.json), [`build_euroboxes.py`](https://github.com/danielrosehill/storage-box-3d-models/blob/main/scripts/build_euroboxes.py). Model asset is CC BY 4.0, generator code MIT. Keep attribution; verify the actual mesh against the declared analytic model. The generic crate is **not** a certified model of Avatar's hardware.

### Proposed technical defaults, not company-confirmed facts

- Host: **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic** for the workbook exercise; later Isaac Sim. This is **not** a claim about Avatar Robotics' deployed stack.
- Reference frame: `base_link` (configurable). Tote frame: `tote`, origin at horizontal center of bottom-most base plane, `+X` along length, `+Y` along width, `+Z` upwards. Transformation `T_A_B` maps coordinates from B into A.
- Baseline tote: **EU6423 outer 0.600 m × 0.400 m × 0.230 m**. The generator's wall thickness is 0.0025 m, floor slab 0.003 m, and stacking foot height 0.015 m. The analytic model and GLB must use identical reference frames and documented exclusions (labels, handles, bevels); verify origin and geometry during ingestion.
- Implementation: C++17 / `ament_cmake` with Eigen, `tf2_ros`, `rclcpp`, standard ROS sensor interfaces and a **small** robust-registration module. Only add PCL or other substantial dependencies if justified by a measurable need.
- Operation: **bounded triggered estimation**, not an unbounded continuous tracker. Begin with 1 fixed camera, then enable 1 and 2 wrist cameras.

## 2. Mission, scope and non-goals

**Mission:** From timestamped, image-derived segmented depth observations and a known *current* tote geometry/pose prior, propose an accurate 6-DOF tote pose in `base_link` with honest uncertainty, quality evidence and model-revision provenance. Later, propose corrections to **observable** geometry parameters without conflating those with pose/calibration changes.

### In scope

- Per-camera mask/depth alignment, depth projection, TF2 transformation at acquisition time, outlier filtering and provenance.
- Bounded multi-view observation collection, robust rigid-model registration, uncertainty and observability checks.
- Candidate estimate with diagnostics; explicit success, rejection and degraded states.
- Independent truth-only evaluator, repeatable simulation and fault tests.
- Phase 2 extension point for parametric geometry correction (not implemented in Phase 1).

### Out of scope

- Training or implementing a general semantic neural network, object detection beyond the tote ROI, robot motion, robot navigation, gripper control, task state machine, production model-manager transaction logic, automatic mesh acceptance, online camera calibration and live dimension changes in Phase 1.
- A model-to-robot safety guarantee. Planner updates and collision clearances are validated downstream by an independent model manager.
- Taking semantic labels from the simulator as runtime inputs. A simple **image-derived** color/shape segmentation baseline is allowed for initial simulation; simulator truth labels are evaluator-only. Unit-test masks may be explicit fixtures, but must not leak test truth during integration evaluation.

## 3. State and frames

Phase 1 state: `T_B_T ∈ SE(3)`, where `B = base_link`, `T = tote`. Fixed geometry parameters `θ₀` are model metadata, **not** optimized. Estimated covariance is a `6×6` local tangent-space covariance; use one explicitly documented perturbation convention and convert it consistently to the published `PoseWithCovariance` ordering `[x,y,z,rotX,rotY,rotZ]` in the reference frame. Never average Euler angles to fuse poses.

Each camera `Cᵢ` has optical frame (ROS convention `z` forward, `x` right, `y` down), `CameraInfo`, calibrated depth and a tote mask. Convert depth to local points `p_C`, then to body points as `p_B = T_B_C(t_acquisition) p_C`. Wrist TF must be available **at capture time**, including joint-state/kinematic interpolation. Refuse missing, too-old, extrapolated or invalid TF; do not use the newest transform as a fallback.

For registered **axial** pinhole depth `Z`: `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy`; undistortion/registration must be verified. **Radial range** cannot be projected using this axial formula. Validate depth convention, units, image encoding and pixels before processing.

Preserve `camera_id`, optical frame, capture stamp, calibration revision/hash, mask provenance, TF lookup/stamp, depth convention, uncertainty assumptions and accepted/rejected observation counts after transforming into the common frame.

## 4. ROS 2 interface contract (proposed v0.1)

### Inputs

Declare camera list in a configuration file, e.g. `head`, `left_wrist`, `right_wrist`; the first MVP enables `head` only. For **each enabled camera**:

- `sensor_msgs/msg/Image` depth topic; support `16UC1` (millimeters) and `32FC1` (meters) **only after validation**. Invalid depth is rejected, not replaced with guessed range.
- `sensor_msgs/msg/CameraInfo` intrinsics and optical-frame ID; reject absent or inconsistent intrinsics.
- `sensor_msgs/msg/Image` single-channel segmentation mask (`mono8`/`8UC1`; document `0=background`, nonzero=`tote`), generated from images by an independent upstream component. Mask dimensions, registration frame and timestamp must match depth within configured tolerance. Do not fabricate segmentation from simulation truth.
- TF2 from each optical camera frame to `base_link` at acquisition stamp.

Model input for MVP: immutable snapshot loaded from `config/tote_model.yaml`, containing `model_id`, `model_revision`, `geometry_hash`, parameter values and prior `T_B_T`/prior covariance. The eventual model manager provides the same immutable snapshot over an interface, not an in-place mutable object; do not invent a dependency on an unavailable service.

### Command

Expose one cancelable `EstimateTote` action (prefer over a blocking service because acquisition/registration may take time). **Conceptual** goal: `object_id` assigned by the task client; `expected_model_revision`; bounded `collection_duration`; `min_distinct_views` (default 1 for MVP); requested mode (`POSE_ONLY`, Phase 2 `POSE_AND_GEOMETRY`); optional caller-provided correlation/threshold profile identifier. Phase 1 must reject unsupported geometry mode explicitly rather than silently ignore it. The action **never moves a camera or robot**.

If the selected target repo prefers another established, equivalent action/interface, document why and preserve these semantics. Goal acceptance must check configured duration/frame/memory limits. Cancellation releases the collection session without publishing a success candidate.

### Outputs

**Candidate output** on `/tote_fusion/candidate` and as action result, with a custom, versioned `TotePoseCandidate` message:

- `std_msgs/Header` (`frame_id=base_link`, timestamp semantics documented)
- `object_id`, `parent_model_revision`, `model_id` and `geometry_hash`
- `geometry_msgs/PoseWithCovariance` for `T_B_T`
- observation interval `[first_capture, last_capture]`, source camera IDs, calibration identifiers
- fit quality (training and **held-out** residuals, inlier fraction, point/sample counts, coverage metrics)
- observability/ambiguity status and (where defensible) singular values or comparable information; covariance convention
- estimation-mode identifier, candidate ID and issue time, validity interval/expiration contract

Do **not** pretend scalar RMSE establishes the accuracy of an occluded surface. Phase 1 candidate references fixed geometry rather than embedding an arbitrary reconstructed mesh.

**Status/diagnostics** on `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) and in the action result/feedback. Include machine-readable failure codes such as `INVALID_INTRINSICS`, `INVALID_DEPTH`, `MASK_MISMATCH`, `TF_UNAVAILABLE`, `STALE_OBSERVATION`, `SIM_TIME_RESET`, `INSUFFICIENT_DATA`, `UNOBSERVABLE`, `AMBIGUOUS_SYMMETRY`, `CAMERA_CONFLICT`, `POOR_FIT`, `STALE_MODEL_REVISION`, `TIMEOUT`, `CANCELED`, `INTERNAL_ERROR`. Messages should state remedial context but not conceal an error behind a success flag.

Publish an acceptable **candidate** only, never the authoritative planning model or an automatic accepted revision. Rejected cases must produce a failed action result and diagnostics; never reuse a prior candidate as if it were current.

### Version ownership

The fusion node reads `model_revision=N` and may propose a candidate with `parent_model_revision=N`; only an external model manager can validate and commit it as `N+1`. A stale candidate must be rejected downstream. No model updates while the node is collecting or optimizing; freeze the snapshot per request. An independent manager must handle atomic commit, plan invalidation and rollback.

## 5. Internal software architecture

Implement one ROS-facing action node plus pure, individually testable components. These are **classes/modules**, not required separate ROS processes:

1. `ObservationAdapter`: align mask/depth/CameraInfo, validate encodings/resolution, back-project to 3D, filter invalid/out-of-range points and self-robot points where configured. Avoid silent unit conversions.
2. `FrameTransformer`: TF2 lookup at image capture time, with max-age/gap/extrapolation checks; emit body-frame observations with unchanged source metadata.
3. `ObservationBuffer`: bounded memory/time/frames; group by camera ID and acquisition interval; deduplicate; record capture gaps, drops, time reset and calibration-change events. Reject cross-reset observations. Do not claim independent measurements merely because many neighboring pixels exist.
4. `ToteGeometry`: immutable analytic surface representation (base, inner and outer wall faces, upper rim, stacking foot as supported). Explicitly mask/ignore features not modeled accurately (raised labels, hand holes, rounded corners). Provide spatial queries and surface normals in tote frame; do not conflate inner and outer extents.
5. `PoseInitializer`: start from prior `T_B_T`; optionally produce alternate edge/corner/rim-based or symmetry-related hypotheses when data supports them; never rely solely on a lucky ICP basin.
6. `RigidRegistration`: robust, deterministic, batch optimization on `SE(3)` (suggest weighted point-to-plane / point-to-surface residual with Huber or comparable robust loss). Use per-camera noise weighting and point subsampling so one dense view does not dominate by pixel count. Keep solver independent of ROS and simulator APIs.
7. `QualityEvaluator`: geometric residuals, independent held-out observations/viewpoints, coverage, consistency among cameras, rank/conditioning, pose uncertainty and ambiguity. A poor/weakly observed fit is rejected even if the solver reports convergence.
8. `CandidateAssembler`: serialize validated candidates and diagnostics without writing to the planner or changing the physical model.

Avoid an unnecessary plugin framework, database, microservice decomposition or parallel algorithm before evidence justifies it. Key dependencies should be injected (clock, transform provider, geometry, solver) to support deterministic unit and failure tests.

## 6. Estimation algorithm: Phase 1

**Objective:** fit observations `{pᵢⱼᴮ}` to visible surfaces of immutable `M(θ₀)`:

`T̂ = argmin_{T∈SE(3)} Σᵢⱼ ρ( d(T⁻¹ pᵢⱼᴮ, M(θ₀)) / σᵢⱼ ) + || Log(T_prior⁻¹ T) ||²_{P_prior⁻¹}`.

This is the suggested baseline; the agent must state point/surface correspondences, residual convention, robust weighting, minimizer, stopping criteria, and prior treatment **before** implementation. It must check support surfaces and occlusion: distance to a nearby back surface should not be used if that surface could not have generated the observed ray. Analyze orientation symmetry (including possible `π` yaw ambiguity) instead of forcing certainty from a symmetric tote.

**Processing order:**

1. Freeze model snapshot and capture interval; clear prior request's transient buffers.
2. Accept valid, synchronized, image-derived mask/depth/CameraInfo observations only; validate units and calibrated depth convention.
3. Lookup time-correct TF and transform points into `base_link`; keep provenance/camera group and sensor error assumptions.
4. Downsample spatially and by view, removing invalid/robot points; allocate disjoint fitting and held-out sets **by view/time where feasible** (not a random split of adjacent correlated pixels alone).
5. Generate prior-based and supported geometric initial pose hypotheses; optimize robust surface residual; reject nonfinite or unconverged solutions.
6. Evaluate coverage, rank/observable DOFs, symmetry and cross-camera disagreement. Explicitly distinguish camera bias from a plausible geometric discrepancy where possible; otherwise report `CAMERA_CONFLICT`/`UNOBSERVABLE`.
7. Estimate a defensible covariance with documented tangent convention, observation correlations and limitations; do **not** treat inverse Hessian alone as calibrated confidence. Singular modes must remain uncertain or cause rejection. Calibrate reported uncertainty against repeated simulation trials.
8. Validate on independent held-out views/noise samples and compare against the unchanged planner prior and a simple unweighted baseline. Emit candidate only if all configured gates pass.

**Failure semantics:** A lost wrist camera may be tolerated if remaining views provide adequate geometric information and policy allows reduced coverage. A shared base calibration fault, insufficient edges, stale frames or ambiguous pose must not be masked by available camera count. No perpetual retry loop; action returns an explicit status. Do not auto-restart controllers or continue robot motion from this node.

## 7. Geometry extension: Phase 2 — width/length implemented under explicit 2026-10-14 authorization

Extend state to `{T_B_T, θ}`, starting with `θ = [L, W, H, wall_thickness]` in meters, adding only observable parameters. Treat **outer dimensions, inner opening and conservative collision envelope separately**. The first controlled extension is tote **width**.

- Pose-only fitting runs first, with fixed dimensions. Geometry fitting is separately enabled and must not covertly compensate for bad TF, camera calibration or an incorrect pose.
- With both lateral edges of a front wall visible and well localized, **width may be measurable from that single view**. Multiple cameras are not inherently required; a wall patch without both edges generally does **not** determine full width.
- Example: planner width `0.400 m`, observed width `0.420 m`; pose-only optimization cannot make both boundaries agree through rigid translation alone. Test width and pose separately, then jointly.
- Example: head camera suggests `0.420 m` but wrist camera `0.405 m`. Do not blindly average; evaluate propagated uncertainty, geometry coverage, calibration and whether the disagreement is consistent with systematic bias. Reject or request another view when uncertain.
- Never shrink unseen collision geometry merely because no points were returned. Geometry candidate includes parameter uncertainty, observed support, unresolved regions and a model generation proposal; authoritative collision mesh publication remains owned by the model manager.
- Avoid free-form mesh warping in the first geometry release; use a constrained parametric geometry model with versioned mesh generation.

## 8. Faults, state transitions, observability and operability

Request-state transitions: `IDLE → COLLECTING → ESTIMATING → VALIDATING → (CANDIDATE_READY | REJECTED | CANCELED | TIMED_OUT) → IDLE`. An incoming duplicate request or overlapping incompatible request is rejected or serialized under explicit policy; no mixed observation sessions or partially overwritten buffers. A ROS simulation-time jump invalidates in-flight time assumptions and forces a fresh collection.

Observability deliverables: ROS diagnostics for active cameras, latest frame/capture stamp, TF lookup status, age/gaps, accepted/rejected frames, valid tote pixels, sampled points, estimated geometric rank/ambiguity, cross-view residuals, candidate model revision and rejection reason. Use structured logs with controlled rate (never print per pixel), correlation/request ID and timestamps. Expose health separately from estimate correctness; the process can be alive while every candidate is invalid.

Configuration: checked YAML values with SI units and documented defaults (`camera_topics`, `body_frame`, `model_file`, `max_observation_age`, `sync_slop`, `tf_lookup_timeout`, `max_capture_window`, `max_frames`, `max_points`, `max_buffer_bytes`, `depth_min/max`, voxel size, robust-loss scale, permitted symmetry handling, rank thresholds, per-camera noise profile, candidate expiry, validation profiles). All numeric gate thresholds are **exercise configuration**, not production safety tolerances. Reject missing/nonphysical/inconsistent parameters on startup and output actionable errors.

Performance instrumentation: define `first_capture → last_capture`, `last_capture → candidate`, `goal_accept → action_result`, and candidate age at handoff. Under specified camera count/resolution/FPS and point budget, report p50/p95/p99 and worst-case response time, success/rejection/drop rate, CPU/RAM peaks and queue depth. Initial profiles could be `1×640×480@15 Hz` and `3×640×480@15 Hz`, but these are **proposed workloads**, not measured or guaranteed targets. Determine deadlines from the actual task before declaring success.

## 9. Required test suite and evidence

Tests must define the **observable oracle**, expected behavior, failure message and mutation likely to be detected. Keep deterministic core tests independent of ROS/Gazebo and record random seeds. Separate a fixed regression set and held-out view/noise set. Simulator truth may be used by a **test-only evaluator** for metrics, never as estimator input.

| ID | Scenario | Required observable behavior (not just solver status) |
| --- | --- | --- |
| T01 | Known axial-depth target and off-axis target | Correct metric projection/frame; mm-to-m and optical convention faults cause assertions to fail. |
| T02 | Fixed-head camera, correct geometry and pose | No systematic drift; reported uncertainty/residual reflects injected noise. |
| T03 | Inject `+20 mm` prior translation, then `+3°` yaw | Measured held-out pose error improves against unchanged prior. Use a nondegenerate/asymmetric scene or explicit symmetry-aware evaluation. |
| T04 | TF inverse/sign bug | Test fails with a known off-axis, nonzero-rotation transform; no identity-only test. |
| T05 | Moving wrist with joint state change / delayed TF | Observation transformed at exposure time; latest-transform substitution fails test. |
| T06 | One planar patch or symmetric geometry | Inadequate DOFs/ambiguity explicitly reported; no spuriously precise 6-DOF claim. |
| T07 | Segmentation outliers and invalid depth | Invalid samples removed; robust fit degrades gracefully or rejects. |
| T08 | Drop one camera / provide conflicting biased views | Permitted degraded mode only if geometry still observable; otherwise rejection; confidence does not artificially increase. |
| T09 | Stale frame, missing CameraInfo, wrong units, mismatched mask | Non-success codes and no candidate publication. |
| T10 | `/clock` reset, cancellation, timeout and bounded-memory overload | Buffers discarded safely; no stale request/result or memory growth. |
| T11 | Duplicate request, stale model revision and concurrent update | No mixed revisions; candidate's parent revision remains immutable; rejected stale action. |
| T12 | Phase 2 fixture: two visible front edges, width 0.420 vs 0.400 m | Width observable; pose-only cannot reduce both-edge residual to zero; Phase 1 does not alter width. Separately enabled geometry mode corrects width within measured tolerance. |
| T13 | Phase 2 fixture: single flat patch, width unknown | Do not invent width or shrink collision envelope; report insufficient geometry. |
| T14 | Phase 2 conflicting camera widths 0.420 and 0.405 m | Disagreement diagnosed; no unconditional averaging; uncertain/reject outcome. |
| T15 | Independent simulator truth boundary | Static/code-level boundary: estimator imports and runtime inputs cannot access truth-only model/labels/poses/IDs. |

**Test evidence:**

- At minimum `ament_cmake_gtest` for geometry, transforms, optimizer, observability, failure policy and revision freezing; ROS-level `launch_testing` or equivalent for action, QoS and cancellation/TF/time behavior.
- Use bag2 or equivalent recordings of real *sensor messages* with expected outcomes when available; do not assume a real-robot known-fault corpus exists. Bag replay is open-loop evidence, not physical validation.
- Mutations: (a) reverse `T_B_C` transform, (b) use newest TF, (c) bypass stale-data rejection, (d) mask out ambiguous pose check, (e) ignore model revision, (f) assume camera disagreements average away. Every mutation must fail at least one relevant test; restoring code must pass. Report which mutants were tested, not a universal mutation score.
- Comparison: unchanged prior, simple unweighted estimator, weighted robust estimator, with sample counts, failure cases, error distributions, and held-out performance. Pass a test only for the claim its oracle actually supports.

## 10. Proposed repository/package layout

```text
<authorized-ros-workspace>/src/
  tote_fusion_interfaces/
    msg/TotePoseCandidate.msg
    action/EstimateTote.action
    CMakeLists.txt  package.xml
  tote_fusion/
    include/tote_fusion/
      observation.hpp  geometry.hpp  registration.hpp
      evaluator.hpp  estimator.hpp  diagnostics.hpp
    src/
      core/                      # pure math/estimation, no ROS ownership
      ros/                       # subscriptions, TF2, action server, adapter
    config/
      tote_model.yaml
      fusion.yaml
      cameras.yaml
    launch/tote_fusion.launch.py
    test/
      test_projection.cpp
      test_frames.cpp
      test_registration.cpp
      test_observability.cpp
      test_fault_policy.cpp
      test_action_contract.py
    docs/
      frames.md  interfaces.md  runbook.md
      design_decisions.md  evaluation.md
    CMakeLists.txt  package.xml  README.md
  tote_fusion_eval/              # OPTIONAL, explicitly test-only truth evaluator
```

Simplify if an interface package is unnecessary for the first vertical slice, but retain a migration path to the specified contract. Avoid creating separate processes for each class. Document public interfaces and frame diagrams; no hardcoded camera topics or magic unit conversions. The `tote_fusion_eval` package may read ground truth; `tote_fusion` must not depend on it in its runtime dependency graph.

## 11. Execution milestones and definition of done

**M0 — environment and source verification**

- Inspect authorized workspace, repo instructions, target ROS/Gazebo versions and dependencies; capture Git commit, branch, `git status --short` and installed versions without modifying unrelated work.
- Inspect EU6423 GLB geometry against `units.json` and geometry generator. Confirm frames, units and mesh origin. Verify actual source license/attribution.
- Create short design-decisions log and frame diagram. Log blockers; do not invent access to hardware or software.

**M1 — first releasable Phase 1 vertical slice**

- One fixed camera, image-derived tote mask, correct depth projection/TF, immutable known geometry, robust pose alignment, pose candidate and diagnostics.
- Controlled pose offset trials with held-out evaluator, frame/depth fault tests and tests that fail under deliberate bad transforms.
- Bounded estimate request, reject reason and no runtime truth access. Reproducible `colcon build`, `colcon test` and a Gazebo launch/recording (or clearly report simulator unavailability and demonstrate offline fixtures instead).

**M2 — multi-view, still Phase 1**

- Enable one then two wrist cameras; preserve acquisition-time TF and camera-specific uncertainty.
- Test camera loss, common biases, conflicting views, queue limits, clock reset and cancellation.
- Record before/after pose error, held-out residual, p50/p95/p99, rejection counts and uncertainty consistency.

**M3 — Phase 2 (requires explicit approval)**

- Estimate width from valid visible boundaries, then add additional independently identifiable dimensions. Produce **geometry candidates**, uncertainty and conservative collision representations; no automatic planner commit.
- Tests for patch-only unobservability, pose/width separation, cross-camera disagreement and prior/calibration bias.

### Mandatory handoff artifacts

1. Documented assumptions and a list of decisions needing user confirmation.
2. Buildable source and lock/pin strategy for container/package dependencies, with README commands.
3. Tests with meaningful numerical/geometric oracles, fixture seeds, fault-injection evidence, including deliberately failing mutations and restorations.
4. Diagnostic example and an example candidate (actual generated output, clearly marked simulated).
5. Evaluation report containing positive and negative cases and honestly scoped claims; reproducible run logs or bag recordings.
6. Minimal runbook: bring up, inspect frames, trigger action, view diagnostics, stop/cancel, reset and recover safely.
7. A final delta summary: what works, what is untested, known failure modes and recommended next single change.

## 12. Open questions / approval gates

**Do not block M1** on missing production information if the proposed simulation defaults suffice; record them. Do block any statement or action that would imply those defaults are actual robot facts.

- Exact target code repo/workspace and whether the agent is authorized to create or modify it. **Do not modify `avatar_challenge` or the knowledge vault by assumption.**
- Real camera topics, models/intrinsics, depth convention, optical frames, TF/extrinsic calibration revisions and moving-wrist time accuracy (unconfirmed).
- Whether segmentation is provided externally or should use a simple image-derived baseline for Gazebo (M1 default: image-derived baseline, not neural segmentation training).
- Actual geometry/CAD license and any difference between the generic EU6423 asset and desired physical tote.
- Selection of model-manager interface after pose-only candidate contract stabilizes; it is an external owner.
- Grasp clearance, pose/dimension tolerances, allowable staleness, covariance calibration and production safety thresholds — **not yet established**.
- Exact simulator and hardware capacity, acceptable p95/p99 estimation time, and whether GPU-based segmentation is required later.

## 13. Quick instruction block for the implementation agent

> Implement **M0 and M1** of `TOTE_FUSION_NODE_SPEC.md`, and stop for design review before M2 or Phase 2. First report the ROS workspace state, versions and source assumptions. Create a well-tested C++17 ROS 2 package with a minimal image-derived segmentation adapter, time-correct depth-to-point transform, immutable tote model and robust 6-DOF pose registration. Produce a versioned **candidate** only; do not update the planner, control a robot, read simulator truth or generate an authoritative collision mesh. Use deterministic nondegenerate unit tests and bounded action/timeout behavior. Demonstrate deliberate transform/timestamp/ambiguity fault tests failing under broken implementations and passing after restoration. Follow the maintainability, reliability, performance and source-of-truth practices cited in this spec. Provide reproducible build/test/launch commands, observed evidence, diagnostics and known limitations. Do not silently add dependencies or alter unrelated repositories.

---

**Provenance note:** This document combines the user's reviewed architecture with an implementation **proposal** informed by the project workbook and the reviewed knowledge-base notes. It is not an approved Avatar Robotics company interface, a statement about real-robot calibration, or evidence of any simulator test being performed.
