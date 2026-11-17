This is the subset of the official Livox SDK `livox_laser_simulation` needed
for MID-360, from commit `1cce1073633a062b92e30243a4c2920e45551bb5`:
https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5

`SHA256SUMS` covers the unmodified upstream files; the upstream MIT license is
in `LICENSE`. `ros1/patches/livox_sim_noetic.patch` applies Noetic/Gazebo 11
compatibility and frame/invalid-return fixes when creating the ROS workspace.
