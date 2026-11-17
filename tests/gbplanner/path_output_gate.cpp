// Boundary regression: interpose only the path improver, to supply an unsafe
// candidate after the original graph path passed. The linked Rrg output entry
// points and TSDF checks remain real. -fno-access-control configures the local
// two-vertex graph without adding test hooks to the production API.
#include <iostream>
#include <ros/ros.h>
#include <voxblox_ros/conversions.h>
#include "gbplanner/rrg.h"

namespace { int improvements = 0; }
bool Rrg::improveFreePath(const std::vector<geometry_msgs::Pose>& original,
                        std::vector<geometry_msgs::Pose>& modified, bool) {
  ++improvements;
  modified = original;
  modified.back().position.x = 1.2;  // Its body reaches the unknown voxel below.
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_output_gate_regression");
  ros::NodeHandle nh, params("~");
  params.setParam("tsdf_voxel_size", .1);
  params.setParam("tsdf_voxels_per_side", 16);
  params.setParam("world_frame", "lio_map");
  params.setParam("update_mesh_every_n_sec", 0.);
  params.setParam("publish_map_every_n_sec", 0.);
  params.setParam("collision_geometry_fix", true);
  Rrg rrg(nh, params);
  rrg.periodic_timer_.stop();
  rrg.global_graph_update_timer_.stop();
  rrg.global_graph_frontier_addition_timer_.stop();
  rrg.camera_annotation_timer_.stop();
  rrg.current_state_.setZero();
  rrg.robot_params_.center_offset.setZero();
  rrg.robot_params_.type = RobotType::kGroundRobot;
  rrg.robot_box_size_ = Eigen::Vector3d(.75, .45, .34);
  auto& p = rrg.planning_params_;
  p.allow_sudden_dir_change = true;
  p.auto_landing_enable = p.auto_homing_enable = false;
  p.path_safety_enhance_enable = true;
  p.path_interpolation_distance = -1.;
  p.edge_overshoot = 0.; p.v_max = .12;
  p.traverse_length_max = 5.; p.traverse_time_max = 5000.;
  StateVec a, b;
  a << 0., 0., .43, 0., 0.; b << .4, 0., .43, 0., 0.;
  auto* start = new Vertex(rrg.local_graph_->generateVertexID(), a);
  auto* goal = new Vertex(rrg.local_graph_->generateVertexID(), b);
  start->is_hanging = goal->is_hanging = false;
  rrg.local_graph_->addVertex(start); rrg.local_graph_->addVertex(goal);
  rrg.local_graph_->addEdge(start, goal, .4);
  rrg.local_graph_->findShortestPaths(rrg.local_graph_rep_);
  rrg.best_vertex_ = goal;

  voxblox::Layer<voxblox::TsdfVoxel> layer(.1, 16);
  for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 0; ++y) {
    auto block = layer.allocateBlockPtrByIndex(voxblox::BlockIndex(x, y, 0));
    for (size_t i = 0; i < block->num_voxels(); ++i) {
      auto& v = block->getVoxelByLinearIndex(i); v.weight = 1.; v.distance = 1.;
    }
  }
  const Eigen::Vector3d unknown(1.25, .05, .45);
  layer.getVoxelPtrByCoordinates(unknown.cast<float>())->weight = 0.;
  voxblox_msgs::Layer message;
  voxblox::serializeLayerAsMsg(layer, false, &message, voxblox::MapDerializationAction::kReset);
  auto pub = params.advertise<voxblox_msgs::Layer>("tsdf_map_in", 1, true);
  const auto deadline = ros::WallTime::now() + ros::WallDuration(5.);
  while (!pub.getNumSubscribers() && ros::WallTime::now() < deadline) {
    ros::spinOnce(); ros::WallDuration(.01).sleep();
  }
  pub.publish(message);
  for (int i = 0; i < 20; ++i) { ros::spinOnce(); ros::WallDuration(.005).sleep(); }
  if (rrg.map_manager_->getVoxelStatus(unknown) != VoxelStatus::kUnknown ||
      rrg.map_manager_->getPathStatus(a.head(3), b.head(3), rrg.robot_box_size_, true) != VoxelStatus::kFree)
    return 2;
  int failures = 0, status = 0;
  if (!rrg.getBestPathSimplified().empty()) {
    ++failures; std::cerr << "FAIL: simplified output returned modified path into unknown\n";
  }
  if (!rrg.getBestPath("lio_map", status).empty()) {
    ++failures; std::cerr << "FAIL: normal output returned modified path into unknown\n";
  }
  if (improvements != 2) {
    std::cerr << "FAIL: path-improvement boundary was not exercised\n"; return 2;
  }
  std::cout << "post-modification output rejection: " << 2-failures << "/2 passed\n";
  return failures ? 1 : 0;
}
