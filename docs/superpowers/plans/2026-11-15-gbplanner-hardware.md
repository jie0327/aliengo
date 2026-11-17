# GBPlanner Hardware Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans, inline implementation with one final independent review.

**Goal:** Prepare a reproducible real-robot deployment entry and a concise Chinese runbook.

**Architecture:** Reuse current planner overlays; isolate real sensor/time/SDK bringup from Gazebo. Use the vendor high-level API only, with explicit SDK/firmware provenance.

**Tech Stack:** ROS1 Noetic, Docker, C++14, Python3, Livox SDK2, vendor AlienGo high-level SDK.

**Spec:** `docs/superpowers/specs/2026-11-15-gbplanner-hardware-design.md`

## Global Constraints

- Preserve planner body .75×.45×.34m, zero added margin, lookahead .25m, cross-track .30m.
- No real robot communication during preparation. Tests use isolated Docker networking and loopback.
- Do not claim hardware validation; SDK version and measured extrinsics remain site inputs.
- No Git commits/worktrees over the current unborn, untracked workspace; keep a local change manifest.

## Review Focus

- Wall time works with no `/clock`; simulation clock stall detection remains effective.
- Driver2 CustomMsg reaches the actual embedded Driver1-compatible LIO subscriber without losing offsets.
- G acceleration and frame conversion happen once, preserving timestamps/covariance units.
- Stale SDK feedback/velocity and handheld mode changes cannot leave a nonzero output latched.
- Documented commands source current overlays and survive unresolved host Catkin symlinks.

## Tasks

- [x] 1. Test then implement real sensor adapter, system-time coordinator support, real configuration and TF launch.
- [x] 2. Test then implement a high-level SDK bridge, compile against selected SDK, and exercise commands via loopback without hardware.
- [x] 3. Prepare camera/runtime image, launcher, read-only readiness check and session recording; verify launch expansion and real MID360 LIO input.
- [x] 4. Write concise deployment guide, link it from README, run relevant/full tests, and obtain independent review with follow-up on fixes.
