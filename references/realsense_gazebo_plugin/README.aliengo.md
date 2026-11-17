This directory contains the unchanged ROS 1 `melodic-devel` source of
[PAL Robotics' RealSense Gazebo plugin](https://github.com/pal-robotics/realsense_gazebo_plugin/tree/49ab134fa517280bb8393e715173dc631ccc2557),
commit `49ab134fa517280bb8393e715173dc631ccc2557`. The source derives from Intel's
RealSense Gazebo work and SyrianSpock's ROS integration; original copyright
headers, authors and upstream README are retained. `source.json` records hashes
of every upstream file. `LICENSE` contains the Apache 2.0 terms from Intel's
original plugin repository.

Copy this package into the ROS workspace before applying
`ros1/patches/realsense_rgbd.patch`. Apply the patch with `patch -p0` from the
directory containing the copied `realsense_gazebo_plugin` directory. Build it
with catkin under ROS Noetic and Gazebo Classic 11. Required packages are
`roscpp`, `gazebo_ros`, `image_transport`, `camera_info_manager`, `sensor_msgs`,
and Gazebo development libraries. Neither librealsense nor a camera driver is
required. Both the original and patched plugin build in
`aliengo-learning:noetic` with GCC 9.4 and Gazebo 11.15.1.
When catkin leaves `CMAKE_BUILD_TYPE` empty, the patch compiles this plugin with
`-O2` to keep per-point RGB projection efficient. Explicit build types, including
`Debug`, retain their caller-selected options. Fast-math is not enabled.

The local patch gives point clouds a dedicated RGB image cache with a capture
pose and mutex. It projects depth optical XYZ through the relative camera poses
into RGB intrinsics, so different RGB/depth resolutions, fields of view and
camera origins are respected. Points outside RGB coverage retain their XYZ and
have black color. Named `r`, `g`, `b` iterators preserve channel order. Cloud
back-projection and CameraInfo share a principal point; CameraInfo has identity
rectification and zero distortion. Depth images use millimeters (`16UC1`),
with finite/range/overflow checks and zero for invalid depth.
Both Gazebo format spellings (`R8G8B8`/`RGB_INT8`, `L8`/`L_INT8`) are accepted;
unsupported formats produce a throttled ROS error instead of terminating Gazebo.

The plugin is a geometric depth renderer. It does not reproduce the D435's
active stereo matching, exposure, material-dependent failures, rolling shutter,
or device-specific calibration. RGB is taken from the latest captured image;
dynamic objects may move between RGB and depth captures. Projection does not
resolve occlusion differences between the two viewpoints. The camera's existing
appearance remains the responsibility of the robot description.

Use plugin name `camera` to create the `/camera` ROS namespace. `prefix` selects
four mandatory Gazebo sensors named `<prefix>depth`, `<prefix>color`,
`<prefix>ired1`, and `<prefix>ired2`. Omit `robotNamespace`: this upstream parser
stops processing tags when it encounters it. The four `*OpticalframeName` tags
set message frame IDs. Sensor poses must match their physical TF frames; optical
children use roll/pitch/yaw `-pi/2, 0, -pi/2`. Image transport derives each
`camera_info` topic from the image topic; the parsed `*CameraInfoTopicName`
settings have no separate effect upstream. Set rates on the sensor tags: the
plugin's update-rate settings govern its Gazebo transport publishers.

Run the native regression with:

```sh
python3 -m unittest discover -s tests -p test_d435_projection.py -v
```

The test copies the original snapshot, applies the actual patch, and compiles
the same projection/depth-conversion header used by the plugin. Its numeric
fixtures exercise different image sizes and fields of view, baseline direction,
rotation, invalid depth, clipping and points outside RGB coverage. It requires
`g++`, `patch`, and Python 3, without ROS or Gazebo.
