#pragma once
#include "CommandGate.h"
#include <string>

namespace aliengo_hardware {
// Same command envelope as the simulation controller. No obstacle policy here.
class HighLevelCore {
 public:
  void feedback(int mode, bool remote, double wall, double now) {
    stateWall_=wall; stateMode_=mode; remote_=remote;
    if (remote || (mode!=0 && mode!=1 && mode!=2)) {
      gate_.setMode(guide_autonomy::Mode::Manual,now);
      sending_=false; ready_=false;
    } else if (gate_.mode()==guide_autonomy::Mode::Stand && wall>standWall_) {
      ready_=mode==1;
    }
  }
  bool fresh(double wall) const { return stateWall_>=0. && wall-stateWall_>=0. && wall-stateWall_<=.4; }
  bool ready(double wall) const { return ready_ && fresh(wall) && !remote_; }
  bool stand(double wall,double now) {
    if (!fresh(wall) || remote_ || (stateMode_!=0 && stateMode_!=1 && stateMode_!=2)) return false;
    gate_.setMode(guide_autonomy::Mode::Stand,now);
    standWall_=wall; ready_=false; sending_=true; return true;
  }
  bool enable(bool value,double wall,double now) {
    if (!value) {
      if (gate_.mode()==guide_autonomy::Mode::Autonomy) stand(wall,now);
      // A stop must clear velocity even when feedback has just disappeared.
      if (gate_.mode()==guide_autonomy::Mode::Autonomy) {
        gate_.setMode(guide_autonomy::Mode::Stand,now); ready_=false;
      }
      return true;
    }
    if (gate_.mode()!=guide_autonomy::Mode::Stand || !ready(wall)) return false;
    gate_.setMode(guide_autonomy::Mode::Autonomy,now); enabledWall_=wall; ready_=false; return true;
  }
  bool receive(double vx,double vy,double wz,double stamp,double now,double wall,const std::string& frame) {
    return gate_.receive(vx,vy,wz,stamp,now,wall,frame);
  }
  guide_autonomy::Velocity sample(double wall,double now) {
    auto velocity=gate_.sample(wall,now);
    if (gate_.mode()==guide_autonomy::Mode::Autonomy &&
        (!fresh(wall) || (!gate_.valid() && wall-enabledWall_>.4))) {
      gate_.setMode(guide_autonomy::Mode::Stand,now); ready_=false;
      standWall_=wall; velocity={};
    }
    txMode_=gate_.mode()==guide_autonomy::Mode::Autonomy && gate_.valid() ? 2 :
            gate_.mode()==guide_autonomy::Mode::Stand ? 1 : 0;
    return velocity;
  }
  std::string mode() const { return guide_autonomy::CommandGate::name(gate_.mode()); }
  int txMode() const { return txMode_; }
  bool sending() const { return sending_; }
  bool valid() const { return gate_.valid(); }
  const std::string& reason() const { return gate_.reason(); }
 private:
  guide_autonomy::CommandGate gate_;
  double stateWall_=-1., standWall_=0., enabledWall_=0.;
  int stateMode_=0, txMode_=0;
  bool remote_=false,ready_=false,sending_=true;
};
}
