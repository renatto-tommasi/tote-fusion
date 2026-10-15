# Robotics advisory documentation review — 2026-10-15

Reviewer: /root/benchmark_advisor, the user-delegated robotics expert.
Scope: Current README, benchmark conclusions, ROS contract and session guidance.
Disposition: Advisory go after correction; no remaining documentation blockers
from this review. No studies were rerun by the advisor.

The initial inspection required four README corrections:

1. Qualify steady-wall bounds as applying after execution initializes the goal,
   and link the admission exception immediately.
2. Separate ACK/response timeout and unexplained persistent BUSY from ordinary
   executing-goal outcomes. Stop new requests, preserve logs/UUID/timing and
   escalate. Do not promise a terminal result or validated automatic restart.
3. Restrict historical bounded-behavior claims to exercised lifecycle paths.
4. Label the previous README review as historical.

The advisor reread the amended README and confirmed all four corrections.
Its final verdict was: “Advisory go for the updated README guidance.”

The conclusions appropriately distinguish complementary geometry gains, noisy-pose
acceptance/accuracy tradeoffs, unusable noisy geometry and wrong accepted shared-bias
poses despite uncertainty coverage. Incomplete geometry, paired comparisons,
accepted-only errors and the unresolved 1% target remain explicit.

The replay disclosures correctly cover isolated contexts, the admission clock
pause, subsequent one-times-speed acquisition/estimation, two-second candidate
validity versus the shipped 0.5 seconds, outside-goal lifecycle overhead and
unchanged production source. This advisory verdict does not certify population
statistics, calibrated uncertainty, production recovery, grasp suitability or
hardware operation. Technical verification has its own independent report.
Editorial approval and ASD-STE100 language review remain pending.
