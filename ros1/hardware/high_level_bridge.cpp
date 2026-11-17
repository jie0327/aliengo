// Vendor AlienGo SDK v3.8.4 high-level only. No LowCmd, stand-up or recovery sequence.
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "HighLevelCore.h"
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

using namespace UNITREE_LEGGED_SDK;
static_assert(sizeof(HighCmd)==113 && sizeof(HighState)==244,
              "Expected the selected AlienGo v3.8.4 ABI; do not substitute another robot SDK");
double wallNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class Bridge {
 public:
  Bridge(ros::NodeHandle& node, UDP& udp): udp_(udp) {
    udp_.InitCmdData(command_);
    commandSub_=node.subscribe("/exploration/cmd_vel",1,&Bridge::command,this);
    standService_=node.advertiseService("/aliengo/controller/stand",&Bridge::stand,this);
    modeService_=node.advertiseService("/aliengo/controller/autonomy",&Bridge::autonomy,this);
    statusPub_=node.advertise<std_msgs::String>("/aliengo/controller/status",1,true);
  }
  void step() {
    const auto previous=udp_.udpState.RecvCount;
    udp_.Recv();
    double wall=wallNow(),now=ros::Time::now().toSec();
    if (udp_.udpState.RecvCount!=previous) {
      HighState state{}; udp_.GetRecv(state);
      if (state.levelFlag==HIGHLEVEL) {
        xRockerBtnDataStruct remote{};
        std::memcpy(&remote,state.wirelessRemote.data(),sizeof(remote));
        bool takeover=remote.btn.value || std::abs(remote.lx)>.1 || std::abs(remote.ly)>.1 ||
                      std::abs(remote.rx)>.1 || std::abs(remote.ry)>.1;
        core_.feedback(state.mode,takeover,wall,now); stateMode_=state.mode;
      }
    }
    ros::spinOnce();
    // Callbacks may have accepted a command newer than the pre-spin time.
    wall=wallNow(); now=ros::Time::now().toSec();
    auto velocity=core_.sample(wall,now);
    // All unused command fields stay at SDK-initialized zero/defaults.
    command_.mode=core_.txMode(); command_.gaitType=0;
    command_.velocity[0]=velocity.vx; command_.velocity[1]=velocity.vy;
    command_.yawSpeed=velocity.wz;
    if (core_.sending()) { udp_.SetSend(command_); udp_.Send(); }
    if (wall-lastStatus_>.05) {
      std::ostringstream json;
      json << std::setprecision(16) << "{\"mode\":\"" << core_.mode()
           << "\",\"stand_ready\":" << (core_.ready(wall)?"true":"false")
           << ",\"command_valid\":" << (core_.valid()?"true":"false")
           << ",\"command_reason\":\"" << core_.reason() << "\",\"command_stamp\":" << stamp_
           << ",\"vx\":" << velocity.vx << ",\"vy\":" << velocity.vy << ",\"wz\":" << velocity.wz
           << ",\"sdk_feedback_fresh\":" << (core_.fresh(wall)?"true":"false")
           << ",\"sdk_mode\":" << stateMode_ << ",\"sending\":" << (core_.sending()?"true":"false") << "}";
      std_msgs::String output; output.data=json.str(); statusPub_.publish(output); lastStatus_=wall;
    }
  }
  void stop() {
    if (!core_.sending()) return;  // Never override a handheld takeover.
    if (core_.mode()!="stand" && core_.mode()!="autonomy") return;
    command_.mode=1; command_.velocity[0]=command_.velocity[1]=command_.yawSpeed=0;
    for (int i=0;i<5;++i) { udp_.SetSend(command_); udp_.Send(); ros::WallDuration(.002).sleep(); }
  }
 private:
  void command(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const auto& t=msg->twist;
    double vx=t.linear.x;
    if (!std::isfinite(t.linear.z)||!std::isfinite(t.angular.x)||!std::isfinite(t.angular.y)) vx=NAN;
    if (core_.receive(vx,t.linear.y,t.angular.z,msg->header.stamp.toSec(),
                      ros::Time::now().toSec(),wallNow(),msg->header.frame_id)) stamp_=msg->header.stamp.toSec();
  }
  bool stand(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& reply) {
    reply.success=core_.stand(wallNow(),ros::Time::now().toSec());
    reply.message=reply.success?"stand requested; await fresh HighState mode=1":"fresh standing/walking feedback required; release handheld controls";
    return true;
  }
  bool autonomy(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& reply) {
    reply.success=core_.enable(req.data,wallNow(),ros::Time::now().toSec());
    reply.message=reply.success?"mode accepted":"completed stand required"; return true;
  }
  UDP& udp_; HighCmd command_{}; aliengo_hardware::HighLevelCore core_;
  int stateMode_=0; double stamp_=0.,lastStatus_=0.;
  ros::Subscriber commandSub_; ros::ServiceServer standService_,modeService_; ros::Publisher statusPub_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"aliengo_high_level_bridge"); ros::NodeHandle node("~");
  if (ros::Time::isSimTime()) { ROS_FATAL("Real SDK bridge requires system time"); return 2; }
  std::string target; int localPort,targetPort;
  node.param<std::string>("/aliengo_hardware/robot_ip",target,"192.168.123.220");
  node.param("/aliengo_hardware/local_port",localPort,8081);
  node.param("/aliengo_hardware/robot_port",targetPort,8082);
  node.getParam("robot_ip",target); node.getParam("local_port",localPort); node.getParam("robot_port",targetPort);
  ROS_WARN("High-level SDK connection to %s:%d; robot must already stand in sport mode",target.c_str(),targetPort);
  UDP udp(localPort,target.c_str(),targetPort,sizeof(HighCmd),sizeof(HighState),2);
  Bridge bridge(node,udp); ros::WallRate rate(500);
  while (ros::ok()) { bridge.step(); rate.sleep(); }
  bridge.stop(); return 0;
}
