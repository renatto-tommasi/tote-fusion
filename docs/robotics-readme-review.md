# Robotics review of the public README

Reviewed 2026-10-14 by a Codex agent acting as a robotics engineering reviewer.
Scope: the public README, its operational instructions and evidence claims;
this is not a new milestone review, hardware certification or safety approval.

**Result: README ready for public publication, with no remaining blocking
documentation findings.** It accurately describes a simulation-only candidate
estimator and the limits of its evidence.

## Checked against the implementation

- The launch arguments, camera catalog, action goal fields and profile/mode
  combinations match the included launch file, node and interfaces. Strict
  profiles reject nominal pi-yaw ambiguity; equivalent profiles require a
  consumer that supports both poses. Minimum views counts camera IDs.
- Input names and head/wrist parameter prefixes match the node. The requirement
  for exact per-camera depth/mask/CameraInfo timestamps, registered axial depth,
  calibrated optical frames and a body-frame ancestor is accurate. Acquisition
  TF requires exact or bounded bracketing support, with no latest-TF substitute.
- Output/action names are absolute and need explicit remapping. The example is
  consistent with the source and the retained successful remapping check in
  `evidence/remapping-check.log`. Namespaces alone do not rename these endpoints.
- Configuration/model snapshots are immutable during operation. Geometry
  proposals expose unresolved parameters, conditional uncertainty and collision
  bounds; they do not certify physical collision clearance or commit a revision.
- External recovery correctly distinguishes missing geometric information,
  calibration conflicts, stale inputs and symmetry. Camera motion requires
  valid capture-time TF. Base motion ends the current stationary-body-frame
  request and requires a refreshed frame-consistent prior before recollection.

The installed launch argument listing was inspected. The published node, launch
file and candidate interface were hash-compared with the reviewed development
source and matched. Standalone build/test logs report 84 checks with zero
errors/failures, including wrappers; this reviewer did not independently rerun
that complete suite or issue a new sensor-backed estimate.

## Visual and evidence checks

All four RViz images were inspected. Before images show one capture per camera
and the nominal prior; after images show six captures per camera. The accepted
after image has a green proposed outer envelope; the rejected after image keeps
the orange nominal reference. The README correctly explains that this is a
union of recorded body-frame observations plus a model overlay, not denoising,
point correction, interpolation of missing object surfaces or an estimator
point-cloud output.

The exported arrays' point counts, three-coordinate shapes, finite values and
accepted/rejected identities were checked against `evidence/clouds/provenance.json`.
The accepted seed 3093 result supplies the reported 620.035 mm length,
420.576 mm width and 1.612 mm held-out point RMSE. Rejected seed 3090 has
`INSUFFICIENT_GEOMETRY`, with no successful candidate. Both admitted 18 frames;
frame admission is correctly distinguished from final estimate acceptance.

The original retained MCAP and report in the development workspace were hashed
and match the public provenance. The exporter projects masked recorded depth
with recorded intrinsics and acquisition-time TF; the viewer only republishes
those exports and draws the prior/candidate. Analytic test masks, injected noise
and the distinction from hardware/Gazebo measurements are stated explicitly.
The public checkout includes compact exports, not the original large recording.

The 60/60 clean and 21/60 noisy M3 acceptance counts match the retained evaluation.
The README reports accuracy conditional on acceptance and keeps synthetic-core,
M2 pose-only and pixel-level M3 evidence separate.

## Finding resolved

The draft claimed that an included library `ModelManager` validated and committed
proposals. No such implementation exists in the two published packages. The
README now correctly assigns validation, commit, model distribution and restart
to an external model manager/consumer and explicitly states that no manager or
commit endpoint is included. The milestone documentation was corrected too.

## Remaining limits

The supplied node requires simulation time and is not hardware ready. Real
segmentation, intrinsic/extrinsic uncertainty, shared calibration/scale faults,
large-motion convergence and physical collision safety remain unestablished.
Height, thickness, underside and hidden defects remain unresolved. The noisy
pixel workload has limited availability; rejection does not prove that every
accepted fit is physically correct. Covariance remains local, conditional and
uncalibrated, and finite workload timings are not production deadline guarantees.

No additional README changes are required for these stated simulation claims.
Public repository creation/upload and runtime deployment are outside this review.

## Addendum — live simulation screenshots, 2026-10-14

The new opening section, Gazebo screenshot and live RViz screenshot were reviewed
against `tools/live-simulation.rviz` and
`evidence/live-simulation-screenshots.json`. No blocking findings were identified;
the README remains ready for publication.

The Gazebo image shows the stationary TIAGo Pro and open EU6423 tote. The RViz
image displays three overlapping scene point clouds with the stated cyan/gold/pink
camera colors, a `world` fixed frame and three nonempty black/white mask panels.
The saved configuration selects the stated cloud/mask topics and colors. The
provenance records receipt of all three 640x480 PointCloud2 streams and nonempty
640x480 `mono8` image-derived masks. Both screenshots and the RViz configuration
match their retained SHA-256 hashes.

The captions correctly distinguish surrounding-scene point clouds from the
estimator's masked-depth inputs. They distinguish this live Gazebo visualization
from the analytic recorded M3 examples and disclose that simulator/robot/
segmentation support belongs to the separate development workspace. Neither
the opening section nor the provenance claims a new estimate, M3 geometry
acceptance or hardware result. These screenshots establish the illustrated live
sensor visualization; they do not independently validate synchronization,
camera calibration, estimator accuracy or collision safety.
