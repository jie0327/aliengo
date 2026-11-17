# GBPlanner turn clearance implementation

Spec: ../specs/2026-11-14-gbplanner-turn-clearance-design.md

1. Back up relevant sources/config/docs. Add a real MapManager C++ test using synthetic TSDF observations and run against old implementation (RED). Add a follower regression for deviations beyond the clearance budget (RED).
2. User revision: repair original oriented body/turn geometry without footprint inflation or extra margin. Preserve original follower limits and per-method unknown-space semantics. Fix artificial prefix terminal yaw and final output rejection. Add reproducible patch/build wiring. Rebuild and run regression (GREEN).
3. Run full host tests and ROS backend checks, then Depot exploration. Resolve actual failures without weakening collision acceptance. Review final changes and update Chinese operation/report/provenance with measured outcomes and limitations.

No Git repository exists here: use logs/gbplanner-turn-clearance/before and progress.md instead of Git worktrees/commits. User has requested implementation; execute inline without further routine approval.
