#ifndef GUIDE_AUTONOMY_COMMAND_GATE_H
#define GUIDE_AUTONOMY_COMMAND_GATE_H
#include <algorithm>
#include <cmath>
#include <string>
namespace guide_autonomy {
enum class Mode { Passive, Stand, Manual, Autonomy };
enum class Request { None, Stand, Autonomy };
struct Velocity { double vx = 0., vy = 0., wz = 0.; };
// ROS independent safety state. Its owner serializes all access with one mutex.
class CommandGate {
public:
    CommandGate(double timeout=.4, double maxVx=.12, double maxVy=.08, double maxWz=.30)
      : timeout_(positive(timeout,.4)), maxVx_(positive(maxVx,.12)),
        maxVy_(positive(maxVy,.08)), maxWz_(positive(maxWz,.30)) {}
    Velocity sample(double wall, double now) {
        if (!std::isfinite(wall) || !std::isfinite(now) || wall < receivedWall_ ||
            wall-receivedWall_ > timeout_ || now < lastStamp_ || now-lastStamp_ > timeout_)
            clear("expired_or_clock_jump");
        return valid_ && mode_ == Mode::Autonomy && !stopped_ ? velocity_ : Velocity();
    }
    bool valid() const { return valid_; }
    const std::string& reason() const { return reason_; }
    bool receive(double vx,double vy,double wz,double stamp,double now,double wall,const std::string& frame) {
        if (stopped_ || mode_ != Mode::Autonomy || pending_ != Request::None) return false;
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
            !std::isfinite(stamp) || !std::isfinite(now) || !std::isfinite(wall) ||
            frame != "base" || stamp <= 0. || stamp <= lastStamp_ || stamp <= transitionStamp_ ||
            now < stamp || now-stamp > timeout_) {
            clear("rejected_command");
            return false;
        }
        velocity_.vx = bounded(vx,maxVx_);
        velocity_.vy = bounded(vy,maxVy_);
        velocity_.wz = bounded(wz,maxWz_);
        lastStamp_ = stamp;
        receivedWall_ = wall;
        valid_ = true;
        reason_ = "fresh";
        return true;
    }
    void setMode(Mode mode, double now) {
        mode_ = mode;
        transitionStamp_ = now;
        lastStamp_ = now;
        pending_ = Request::None;
        clear("mode_transition");
    }
    void stop() { stopped_ = true; pending_ = Request::None; clear("stopped"); }
    void request(Request request) { if (!stopped_) { pending_ = request; clear("mode_request"); } }
    void cancelAutonomy() {
        if (!stopped_) {
            // A delayed disable cannot override a newer keyboard/manual mode.
            pending_ = mode_ == Mode::Autonomy ? Request::Stand : Request::None;
            clear("autonomy_cancelled");
        }
    }
    Request takeRequest(bool standReady) {
        Request result = pending_;
        pending_ = Request::None;
        if (result == Request::Autonomy && (mode_ != Mode::Stand || !standReady)) return Request::None;
        return result;
    }
    Mode mode() const { return mode_; }
    static const char* name(Mode mode) {
        switch(mode) {
        case Mode::Passive: return "passive";
        case Mode::Stand: return "stand";
        case Mode::Manual: return "manual";
        case Mode::Autonomy: return "autonomy";
        }
        return "passive";
    }
private:
    static double positive(double value, double fallback) { return std::isfinite(value) && value>0. ? value : fallback; }
    static double bounded(double value,double limit) { return std::max(-limit,std::min(limit,value)); }
    void clear(const char* reason) { velocity_=Velocity(); valid_=false; reason_=reason; }
    Mode mode_ = Mode::Passive;
    Request pending_ = Request::None;
    bool valid_ = false, stopped_ = false;
    double lastStamp_ = 0., transitionStamp_ = 0., receivedWall_ = 0.;
    double timeout_,maxVx_,maxVy_,maxWz_;
    Velocity velocity_;
    std::string reason_ = "no_command";
};
}
#endif
