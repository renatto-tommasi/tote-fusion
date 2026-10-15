# Paired camera benchmark

[Pilot results and retained evidence](results-20261015.md): 600 cached scenes,
14,400 sensor requests, no general fusion-superiority claim.
[Fresh confirmation and native ROS results](results-confirmation-20261015.md):
1,200 new numerical scenes, 30,000 rows and 540 isolated ROS requests.
Complementary geometry improves; noisy pose and shared bias prevent a general
fusion-superiority or calibrated-confidence claim.

[Contract and provisional criteria](contract.md) fix the comparisons before
results. The production core is unchanged from434803c. The primary is the shipped
declared-noise/Huber solver. An isolated build supplies equal-precision ablations;
it is not an installed registration fallback or a new production profile.

## Environment and build

Use the repository's assertions-enabled build. Install system Python NumPy,
SciPy and PyYAML (`python3-numpy`, `python3-scipy`, `python3-yaml`). The commands
use `python3 -s` to select the compatible system scientific stack. The host's
user-site NumPy2.4/system SciPy1.11 mixture failed ABI import and is not used.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tote_fusion_interfaces tote_fusion --symlink-install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g'
python3 -s -m unittest discover -s benchmarks -p 'test_benchmark.py' -v
```

`benchmark_declared` links the production core. `benchmark_equal` links a generated
source copy with only TWO precision denominators changed to the first configured
camera's reference variance. Exact source seams must match or generation fails.
Sigmas still set Huber standardization and quality gates in every method. Squared
loss uses the existing large Huber transition, with all other configuration fixed.
No production registration code or release default is changed.

The robotics advisor checked that the `1e12` transition is exactly quadratic for
this bounded corpus: gated surface residual/sigma is at most26.7. Pixel-derived
boundary sigma is at least0.003m. Finite scene coordinates, bounded dimensions
and at most160 steps below0.5m give a conservative boundary residual below100m,
far below that transition. This reasoning applies to these fixtures; the generic
boundary API permits any positive sigma and has no global quadratic-mode promise.

## Cache once, replay subsets

```bash
mkdir -p benchmarks/runs
python3 -s benchmarks/paired.py generate --output benchmarks/runs/pilot-cache \
  --scenes 30 --first-seed 100000 --partition pilot
python3 -s benchmarks/paired.py run --cache benchmarks/runs/pilot-cache \
  --output benchmarks/runs/pilot-results
```

Outputs must be new paths. Raw caches/results remain local under the Git-ignored
`benchmarks/runs`. Never overwrite sealed evidence. For confirmation, freeze the
pilot configuration and use separate seeds:

```bash
python3 -s benchmarks/paired.py generate --output benchmarks/runs/fresh-cache \
  --scenes 200 --first-seed 200000 --partition confirmation \
  --conditions noise shared_bias weak_complementary
python3 -s benchmarks/paired.py run --cache benchmarks/runs/fresh-cache \
  --output benchmarks/runs/fresh-results
```

Each condition has independently varied scene IDs. Physical-scene seeds are paired
across conditions where useful, so do not pool conditions as independent samples.
Pilot seeds are restricted to[100000,200000), confirmation to[200000,300000).
Repeated frames/methods are never counted as additional independent scenes.

Each cached scene includes `pixels.npz`, `inputs.json` and separate `truth.json`.
A digest covers all input pixel bytes, transform matrices, calibration metadata,
prior and configuration. Truth has its own hash and is never loaded by the C++
worker. An input or truth mismatch stops replay. The independent finite-box
ray/slab renderer includes the opening/handles/feet and nearest-hit occlusion,
finite camera field and depth range. It does not call the estimator raycaster.
Its masks are explicit oracle test masks, not production segmentation.

Pose scenes have true nominal600/400mm geometry. Geometry scenes vary width/length
by12–25mm. `correct_prior` fixes the POSE prior to truth; in geometry mode the
nominal size prior intentionally remains wrong. The complementary condition
restricts A to a weak floor-only pose view or a width wall without floor support;
B adds missing surfaces. Crops are mask ROI restrictions, not physical edges.
The occlusion case uses an actual foreground box. Noise is applied to axial Z
before the supplied transform; camera/shared calibration faults perturb declared
transforms without changing physical images. All six acquisition times are shared.

The worker uses production `project` and `image_boundaries`, so geometry
qualification starts from pixels rather than supplied analytic endpoints.
Fitting and held-out frames alternate in shared100ms time blocks. A/B retain
camera indices0/1 and TWO configured sigma slots in every subset. Shrinking the
sigma vector would change the shipped normalization and covariance multiplier.

Equal-time receives the same capture schedule; A+B gets the union. Equal-sample
uses at most300 surface samples per fitting/held-out partition, limited by the
smaller nonempty per-camera available count. Boundary budgets use the smaller
nonempty observed count separately. All budgets are chosen BEFORE optimization. A/B receive that budget; A+B
splits it equally by camera. Proportional deterministic thinning preserves
observed frame/axis/side strata. Missing support is not replenished; report the
actual counts and resulting rejection. The300-surface limit equalizes capped
surface strength in the static six-frame fixtures. Moving views can have unequal
frame populations: proportional selection can still exceed100 in one frame and
lose a small amount at the cap. Selected potential cap budgets are exported, and
that residual difference must remain visible. Dropout also retains its shortage.
Boundary group caps still differ when views
supply distinct axes/lines; equal raw counts do not equalize joint information. Selected-data hashes must match across all four
precision/loss ablations.

## Read results

`requests.jsonl` contains one row per subset/budget/factor request plus one prior
reference per scene. Every rejection remains present with null accuracy. Accepted
rows preserve full covariance, observable axes, symmetry choice, errors, coverage,
selection hashes and extraction/fit time. Prior covariance is converted from its
supplied tangent coordinates; no dimension confidence is invented.

`summary.json` reports condition-specific acceptance, spatial usability, false
acceptance denominators and exact intervals, median/p95 errors, pose/joint coverage,
and scene-paired bootstrap comparisons. Missing required geometry is incomplete,
not a zero error. The primary1% false-acceptance target remains inconclusive with
only200 accepted/zero wrong; at least299 accepted/zero wrong are needed for its
one-sided95% upper bound. Secondary sensitivity never replaces primary failures.

Pixel-core timing includes extraction and fitting, both hypotheses and geometry
stages. It is not ROS action latency or measured estimate age. End-to-end usability
is null. The `tf_gap` cache contains moving camera inputs and a recorded ROS fault
intent; the numerical core consumes explicit matrices and does NOT exercise TF
admission. Actual transport, TF/timestamp faults, dropout policy, action timing
and capture-to-publication age require separately identified same-byte ROS replay.
Never describe pixel-core results as proof of those node behaviors or hardware
picking safety. No superiority claim is implied by generating this report.

Method execution order is fixed and timings share one host. Do not interpret small
compute-time differences as a latency advantage. A dedicated timing study needs
randomized method order and controlled host load.

Use the [operating procedure](procedure.md) for exact commands and failure handling.
The [session record](session-20261015.md) records evidence, review and remaining work.

## Native ROS replay

The [ROS contract](ros-contract.md), [operating procedure](ros-procedure.md) and
[B2 session record](session-ros-20261015.md) describe the same-byte native replay.
It retains two configured camera slots and uses the shipped declared-precision,
Huber solver. Equal-sample and precision/loss ablations remain numerical-only.
Each request has a fresh evaluator context and native process. A positive clock
pause establishes admission before one-times-speed acquisition and estimation;
the full pause is included in action latency. The explicit replay profile uses
two-second candidate validity, compared with the shipped 0.5-second default.

The first persistent-context preflight stopped on a goal-response transport
timeout. Fixed-count reused/unique endpoint diagnostics did not resolve it.
Fresh contexts passed screening and independent controls. They do not repair
the separate production reservation recovery gap or establish reliability of
a continuously running client. Failed attempts remain separate from population
results; the session preserves their exact scope and provenance.
