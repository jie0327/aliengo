#include "planner_common/params.h"
#include <cassert>
#include <iostream>

int main() {
  SensorParamsBase sensor;
  sensor.type=SensorType::kLidar; sensor.camera_type=CameraType::kFixed;
  sensor.min_range=.5; sensor.max_range=8;
  sensor.center_offset={.25,0,.145};sensor.rotations.setZero();
  sensor.fov={2*M_PI,1.09};sensor.resolution={.17,.087};
  sensor.use_vertical_fov_limits=true;sensor.vertical_fov_limits={-.126,.964};
  sensor.updateFrustumEndpoints();
  StateVec state=StateVec::Zero();
  auto inside=[&](double angle) {
    Eigen::Vector3d point=sensor.center_offset+Eigen::Vector3d(3*std::cos(angle),0,3*std::sin(angle));
    return sensor.isInsideFOV(state,point);
  };
  assert(inside(.95));assert(inside(-.12));assert(!inside(-.20));assert(!inside(1.0));
  assert(!sensor.isInsideFOV(state,sensor.center_offset));
  sensor.rotations[1]=.3;sensor.updateFrustumEndpoints();
  state << 1,2,.43,.7,.1;
  std::vector<Eigen::Vector3d> endpoints;sensor.getFrustumEndpoints(state,endpoints);
  assert(endpoints.size()>400);
  Eigen::Matrix3d body=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY())).toRotationMatrix();
  Eigen::Vector3d origin=state.head<3>()+body*sensor.center_offset;
  for(auto p:endpoints) {
    assert(std::abs((p-origin).norm()-8)<1e-7);
    assert(sensor.isInsideFOV(state,p));
  }
  std::cout << "asymmetric MID360 gain angles, offset, native range and body rotation: PASS\n";
}
