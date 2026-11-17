// Exercise the linked MapManager against synthetic TSDF voxels via its ROS map
// input. No private-field access, Gazebo, or robot truth enters the planner.
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <ros/ros.h>
#include <voxblox_ros/conversions.h>
#include "map_manager/map_manager_voxblox_impl.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "turn_clearance_regression");
  ros::NodeHandle nh, params("~");
  params.setParam("tsdf_voxel_size", .1);
  params.setParam("tsdf_voxels_per_side", 16);
  params.setParam("world_frame", "lio_map");
  params.setParam("update_mesh_every_n_sec", 0.);
  params.setParam("publish_map_every_n_sec", 0.);
  params.setParam("collision_geometry_fix", true);
  params.setParam("occupancy_distance_voxelsize_factor", .5);
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> map(nh, params);
  map.setBoxCheckMethod(0);
  map.setLineCheckMethod(1);
  auto pub = params.advertise<voxblox_msgs::Layer>("tsdf_map_in", 1, true);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5.);
  while (pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) {
    ros::spinOnce(); ros::WallDuration(.01).sleep();
  }
  if (!pub.getNumSubscribers()) return 2;
  int failures = 0, checks = 0;
  auto expect = [&](bool condition, const std::string& label) {
    ++checks;
    if (!condition) { ++failures; std::cerr << "FAIL: " << label << '\n'; }
  };
  const Eigen::Vector3d body(.75, .45, .34), center(.04, .09, .43);
  auto load = [&](const Eigen::Vector3d& pole, bool unknown = false) {
    voxblox::Layer<voxblox::TsdfVoxel> layer(.1, 16);
    for (int x = -2; x <= 1; ++x) for (int y = -2; y <= 1; ++y)
      for (int z = -1; z <= 0; ++z) {
        auto block = layer.allocateBlockPtrByIndex(voxblox::BlockIndex(x, y, z));
        for (size_t i = 0; i < block->num_voxels(); ++i) {
          auto& voxel = block->getVoxelByLinearIndex(i);
          voxel.weight = 1.; voxel.distance = 1.;
        }
      }
    auto* voxel = layer.getVoxelPtrByCoordinates(pole.cast<float>());
    voxel->weight = unknown ? 0. : 1.; voxel->distance = 0.;
    voxblox_msgs::Layer msg;
    voxblox::serializeLayerAsMsg(layer, false, &msg, voxblox::MapDerializationAction::kReset);
    pub.publish(msg);
    for (int i = 0; i < 20; ++i) { ros::spinOnce(); ros::WallDuration(.005).sleep(); }
    expect(map.getVoxelStatus(pole) == (unknown ? VoxelStatus::kUnknown : VoxelStatus::kOccupied),
           "fixture TSDF is loaded through actual ROS map input");
  };
  load({.05, .45, .45});
  expect(map.getRotatingBoxStatus(center, body, 0., M_PI/2, true) == VoxelStatus::kOccupied,
         "90-degree trunk front overlaps pillar outside old world-y half-width");
  load({-.05, -.45, .45});
  expect(map.getRotatingBoxStatus({-.04, -.09, .43}, body, 0., -M_PI/2, true) == VoxelStatus::kOccupied,
         "negative-coordinate turn retains front clearance");
  load({.35, .05, .45});
  expect(map.getPathStatus({0., 0., .43}, {.6, 0., .43}, body, true) == VoxelStatus::kOccupied,
         "swept volume includes interior pillar, not only four surfaces");
  load({.45, .05, .45});
  expect(map.getPathStatus({0., 0., .43}, {.12, 0., .43}, body, true) == VoxelStatus::kOccupied,
         "short segment includes body overhang beyond endpoint");
  map.setLineCheckMethod(0);
  load({.25, .05, .45});
  expect(map.getPathStatus({0., 0., .43}, {0., 0., .43}, body, true) == VoxelStatus::kOccupied,
         "zero-length path still checks actual body footprint");
  expect(map.getPathStatus({0., 0., .43}, {.12, 0., .43}, body, true) == VoxelStatus::kOccupied,
         "sub-metre method-zero edge has no metre-sized sampling gap");
  load({.05, .45, .45}, true);
  expect(map.getRotatingBoxStatus(center, body, 0., M_PI/2, true) == VoxelStatus::kUnknown,
         "unknown turn clearance is not fabricated free space");
  expect(map.getRotatingBoxStatus(center, body, M_PI/2, M_PI/2, true) == VoxelStatus::kUnknown,
         "stationary turn rejects unknown swept space");
  map.setLineCheckMethod(1);
  expect(map.getPathStatus(center, {.04, .2, .43}, body, false) == VoxelStatus::kFree,
         "official relaxed line check still permits unknown space when requested");
  map.setLineCheckMethod(0);
  expect(map.getPathStatus(center, {.04, .2, .43}, body, false) == VoxelStatus::kUnknown,
         "official method-zero box result still propagates unknown");
  expect(map.getPathStatus({std::numeric_limits<double>::quiet_NaN(), 0., .43},
                          center, body, false) == VoxelStatus::kUnknown,
         "invalid geometry cannot be normalized into relaxed free space");
  map.augmentFreeBox(center, body);
  expect(map.getVoxelStatus({.05, .45, .45}) == VoxelStatus::kUnknown,
         "turn envelope does not enlarge root clearing into unknown space");
  load({.55, .05, .45});
  expect(map.getBoxStatus({.15, .05, .43}, body, true) == VoxelStatus::kOccupied,
         "fractional centre checks all intersecting boundary voxels");
  load({-.45, 1.05, .45});
  expect(map.getPathStatus({0., 0., .43}, {.6, .6, .43}, body, true) == VoxelStatus::kFree,
         "diagonal sweep excludes empty bounding-prism corners");
  load({.05, .05, .05});
  expect(map.getPathStatus({0., 0., .43}, {.4, 0., .43}, body, true) == VoxelStatus::kFree,
         "measured floor below body is not inflated into collision");
  load({.35, .35, .45});
  expect(map.getPathStatus({0., 0., .43}, {.2, 0., .43}, body, true) == VoxelStatus::kFree,
         "original-width straight passage is not blocked by full-turn enclosure");
  expect(map.getRotatingBoxStatus({0., 0., .43}, body, 0., .10, true) == VoxelStatus::kFree,
         "small heading adjustment checks its actual angle, not all 360 degrees");
  expect(map.getRotatingBoxStatus({0., 0., .43}, body, 0., M_PI/2, true) == VoxelStatus::kOccupied,
         "large heading adjustment still rejects actual corner-post overlap");
  load({1.05, 1.05, .45});
  expect(map.getBoxStatus(center, body, true) == VoxelStatus::kFree,
         "observed open turn remains usable");
  expect(map.getPathStatus({0., 0., .43}, {.4, 0., .43}, body, true) == VoxelStatus::kFree,
         "observed clear short segment remains usable");
  expect(map.getPathStatus({0., 0., .43}, {0., .4, .43}, body, true) == VoxelStatus::kFree,
         "axis-aligned positive-y segment remains usable");
  std::cout << "turn clearance: " << checks-failures << '/' << checks << " passed\n";
  return failures ? 1 : 0;
}
