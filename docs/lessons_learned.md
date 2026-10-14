# Milestone lessons

These are implementation lessons tied to retained evidence, not hardware claims.
Update this log after each milestone and after independent review.

## M0 — environment and sensor preparation

- Verify the actual renderer's axial depth, optical axes and principal point.
  The backend needed explicit frame/principal-point normalization; applying it
  again in the estimator would introduce a second error.
- Keep truth, public planning state and sensor inputs separate from the first
  fixture. A ground-truth mask would invalidate an otherwise accurate demo.
- There is no workspace-level Git repository. Preserve source snapshots and
  hashes instead of reporting a nonexistent commit. Pin upstream repositories.

## M1 — fixed geometry, stationary head camera

- A nondegenerate offset test exposed excessive prior influence when a whole
  frame was normalized to one effective sample. Use declared information caps
  and compare against the unchanged prior; low residual alone is inadequate.
- Repeated pixels and nearby frames are correlated. Local inverse-Hessian
  covariance needs explicit floors and an independently perturbed population.
  Calibration seeds must be separate from validation seeds.
- Contaminating only a floor subset did not establish the intended rejection
  oracle. Perturb every intended surface family and demonstrate named mutations.
- A rectangular tote has a pi-yaw equivalence class after labels are excluded.
  A prior chooses a local basin; it cannot establish unique global orientation.
- Body-to-world conversion belongs in the independent evaluator and must use
  capture time. Truth errors and held-out sensor residuals measure different
  things; shared calibration error can produce a low residual and wrong pose.
- Rebuilding ROS message definitions changes binary compatibility. Historical
  binaries require their matching interface snapshot; retain that snapshot.

## M2 — multiple cameras and moving capture-time transforms

- Queue keys must contain both camera identity and timestamp. Simultaneous
  camera captures are distinct observations, not duplicates.
- Split fitting/validation by shared time blocks across cameras. Different
  cameras at the same time do not create an independent validation interval.
- Cap information per camera so repeated frames or denser pixels cannot make
  that camera dominate or create false confidence. Camera loss must be explicit.
- Interpolation needs transforms on both sides of acquisition. Verify the
  interpolated transform numerically, as well as its difference from latest TF;
  a transform-presence check alone does not establish the correct pose.
- Synchronize the simulated clock before accepting a new request. An optimized
  solver exposed a fixture race that a slower Debug build had hidden.
- Keep rejected slow trials. The three-camera Debug integration expired and
  correctly produced no candidate; optimized assertions-enabled compilation
  restored useful timing without relaxing the freshness bound.
- Shared run-level calibration perturbations must remain shared across held-out
  frames. Those frames cannot independently validate the shared bias.
- Specify stochastic correlations precisely. Moving a 0.75 mm frame perturbation
  from shared time frames to independent camera/frame pairs changed rejection
  and covariance results. Recalibrate from calibration seeds, retain the earlier
  population, then assess fixed validation seeds without tuning to them.
- Bounded nonconvergence is a valid conflict outcome. A rotational-disagreement
  negative need not fabricate a specific conflict diagnosis; accept only the
  relevant explicit rejection classes and retain the observed exact code.
- A finite-sample p99 is descriptive. State image sizes, active/configured
  camera counts, capture schedule and build flags with every timing report.
- An analytic moving-sensor replay verifies timing and estimator behavior; a
  separate live Gazebo trial verifies image-derived segmentation. Neither
  establishes hardware calibration or actuated robot performance.

- Review camera participation against raw inputs, not only successful
  correspondences. A camera with zero supported rays must not disappear from
  validation or count as an acceptable second view.
- TF2 can interpolate an eight-second bracket without error. Bound actual
  supporting records along dynamic ancestors; a successful lookup is not a
  freshness oracle. Preserve static/topology identity separately from motion.
- Polling can miss a processed reset followed by immediate clock catch-up.
  Use event notification and serialize reset signaling with publication. Never
  read the ROS clock under the publication mutex: clock callbacks can already
  hold its internal lock.
- Keep sealed evidence immutable. Reproduction commands must create new
  directories, and review fixes need their own source snapshot and workload.

## M3 — observable geometry proposals

- A mask boundary is not a physical boundary. Require positive background,
  local wall support and opposing lines; equal-depth crops must fail a mutation
  that disables the free-space gate.
- Test coplanarity using measured normals. A 3-degree wrong prior made true
  opposing endpoints differ in the prior's wall coordinate; using that axis
  as a physical coplanarity oracle incorrectly rejected valid support.
- Exclude the silhouette column from plane support. Its adjacent thin wall can
  skew the PCA normal enough to fail a genuine pair. Use inward neighborhood
  support wide enough for the declared depth noise, with unchanged quality gates.
- Wrong fixed dimensions can cause bounded correspondence oscillation on
  opposing surfaces. A separately requested two-stage geometry algorithm must
  report finite fixed-stage status, then require joint convergence. Do not catch
  a general failure and retry; nonfinite/support/cancellation faults still abort.
- Keep parameter measurements separate from full-pose observability. A wall's
  paired endpoints can establish width without establishing tote height/pose Z.
- Regenerate nominal boxes rather than scaling them. Uniform scaling changes
  thickness, handle openings and foot insets, inventing unobserved geometry.
- Full covariance must retain pose/size cross terms. Legacy pose confidence
  calibration does not calibrate a larger joint state.
- Shared rigid error can preserve dimensions and bias pose. Common lateral/focal
  scale can imitate size; sensor-only low residual cannot diagnose that ambiguity.
- When extracting a shared test fixture, update all injection seams. Capturing
  a renderer in a class attribute means replacing a module global no longer
  injects noise. The corpus now changes and restores the shared renderer seam.
- Header-only fixture functions need inline definitions when used in multiple
  test translation units; splitting tests exposed the existing one-file assumption.


- The initial crop-bypass mutant survived because a different association gate
  rejected the severe crop. Move the fixture's crop inside the allowed basin so
  it isolates the intended free-space gate; retain the survivor and final kill.
- Dimension agreement does not imply endpoint placement agreement. A 12 mm
  wall-parallel camera shift preserved width and hid behind density/sigma in
  global RMSE. Gate every camera/axis/side on fitting and hold-out independently.
- Development shell commands must stop after a failed build. A test executable
  from the previous source can otherwise run and print passing results. Retain
  the failed build and explicitly replace that invalid evidence with a fresh
  compiled numerical oracle; mutation runners already use checked subprocesses.

- Independent review found that scalar-width fixture helpers placed paired
  endpoints on the nominal orthogonal wall even when both true dimensions
  changed. Pass complete physical dimensions to the shared fixture. Correct
  wall coordinates matter to joint pose Jacobians, even when lateral-only
  dimension residuals let an inconsistent fixture pass. Preserve the old
  population and recompute consistent evidence before handoff.

- Do not use zero defaults for missing accepted-trial error fields in evidence
  summaries. Filter explicit rejections, then require accepted measurements by
  indexing. Reproduce the original silent zero and the corrected loud failure;
  complete-data output must remain identical. Keep a supplemental source fix
  rather than rewriting sealed runtime evidence.
