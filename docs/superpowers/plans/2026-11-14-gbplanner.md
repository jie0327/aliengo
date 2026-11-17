# GBPlanner Exploration Implementation Plan

**Goal:** Repeated GBPlanner3 planning and low-speed AlienGo exploration after manual mapping, unchanged horizontal MID360 and lower D435.

**Current spec:** `docs/superpowers/specs/2026-11-14-gbplanner-design.md`.

**User override:** use official spatial checking only; remove the previously added root/full-footprint/low-band/full-returned-path/prefix checks. Command lifecycle, stop and data freshness remain. Old plan is archived at `logs/gbplanner-integration/before/plan-custom-checks.md`.

## Constraints

- Keep sensor poses unchanged; preserve native ray origins/stamps, current Super-LIO only.
- Manual initialization1.5–2m while fusion already runs.
- Official Rrg/root initialization/path callback unchanged; no `/exploration/check_path` service or terrain-footprint enable gate.
- Observed elevation only; no infinite floor or truth navigation.
- No Git repository; work in place, keep upstream references and separate patches. Old workspace read-only.
- Heavy builds/simulations serialized with compute guard.

## Task 1: Official backend and native fusion

- [x] Pin original source/dependencies; build native multisensor TSDF and actual gain geometry.
- [x] Reproduce/fix cardinal-ray defect with six-axis and rotated-camera regressions.
- [x] Restore official Rrg and planner callback byte-for-byte; delete custom check service/helpers/tests and prefix work.
- [x] Rebuild and verify native origins/stamps/TF retry and direct official planner service.

## Task 2: Mapping inputs

- [x] Measured-floor GridMap, unknown NaN, native cloud queues, observed history and diagnostics.
- [x] Actual C++ GridMap roundtrip, ROS adapter and IMU-to-base pose/twist checks.
- [x] Preserve pre-floor observations and bounded timestamp-matched gyro handling.

## Task 3: Guide control

- [x] Stamped bounded velocity, command expiry and no conflicting odometry TF.
- [x] Control-thread services/status; cancellation preserves keyboard Passive/Manual.
- [x] Reproducible patch, focused regressions and final controller build.

## Task 4: Coordinator and entry points

- [x] Current overlay loading, sim/start/enable/stop scripts and RViz.
- [x] Short execution/follow/replan, generation cancellation, RPC/mode/JSON regressions.
- [x] Remove custom spatial gates and guard workers; retain only official path judgement and command/data lifecycle.
- [x] Verify revised actual ROS boundary tests, including command-heartbeat interruption.

## Task 5: Actual verification and docs

- [x] First custom-check trial OmDA1F retained as FAIL: manual1.685m, root accepted, upstream2.9–3.26m path discarded before execution. Historical only.
- [x] Update docs to latest official-check implementation and verify no stale interface requirements.
- [x] Run revised actual Gazebo acceptance: measured warmup, official nonempty paths, repeated execution and >=1m motion.
- [x] Record both-source/map growth, posture, command loss, clock pause/recovery, explicit stop/new generations.
- [x] Fresh scoped review and appropriate project tests, then report measured limits.
