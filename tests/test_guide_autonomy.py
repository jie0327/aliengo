"""Compile and exercise the real C++11 command gate without ROS/Gazebo."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class GuideCommandTests(unittest.TestCase):
    def test_real_command_gate(self):
        source = r'''
#include "autonomy/CommandGate.h"
#include <cassert>
#include <cmath>
#include <limits>
using namespace guide_autonomy;
int main() {
    CommandGate g;
    assert(g.sample(1., 10.).vx == 0. && !g.valid());
    assert(!g.receive(1., 0., 0., 10., 10., 1., "base"));
    g.setMode(Mode::Autonomy, 10.);
    assert(!g.receive(NAN, 0., 0., 10.1, 10.1, 1., "base"));
    assert(!g.receive(0., INFINITY, 0., 10.1, 10.1, 1., "base"));
    assert(!g.receive(0., 0., NAN, 10.1, 10.1, 1., "base"));
    assert(!g.receive(.1, 0., 0., 10.1, 10.1, 1., "odom"));
    assert(!g.receive(.1, 0., 0., 9., 10.1, 1., "base"));
    assert(!g.receive(.1, 0., 0., 11., 10.1, 1., "base"));
    assert(g.receive(2., -2., 2., 10.1, 10.1, 1., "base"));
    Velocity v = g.sample(1.1, 10.1);
    assert(std::abs(v.vx-.12)<1e-9 && std::abs(v.vy+.08)<1e-9 && std::abs(v.wz-.30)<1e-9);
    assert(!g.receive(.1, 0., 0., 10.1, 10.1, 1.3, "base"));
    assert(g.sample(1.41, 10.1).vx == 0. && !g.valid());
    // A paused publisher cannot refresh a watchdog with repeated source stamps.
    assert(!g.receive(.1, 0., 0., 10.1, 10.1, 2., "base"));
    assert(g.receive(.1, 0., 0., 10.2, 10.2, 2., "base"));
    assert(g.sample(2.1, 9.).vx == 0.); // clock rewind invalidates
    g.setMode(Mode::Stand, 10.2);
    assert(!g.receive(.1, 0., 0., 10.3, 10.3, 3., "base"));
    g.setMode(Mode::Autonomy, 10.3);
    assert(!g.receive(.1, 0., 0., 10.2, 10.3, 3., "base"));
    assert(g.sample(3., 10.3).vx == 0.);
    assert(g.receive(.1, 0., 0., 10.4, 10.4, 3., "base"));
    g.setMode(Mode::Autonomy, 10.4); // every transition clears, even same public mode
    assert(g.sample(3., 10.4).vx == 0.);
    assert(g.receive(.1, 0., 0., 10.5, 10.5, 3., "base"));
    g.request(Request::Stand);
    assert(!g.receive(.1, 0., 0., 10.6, 10.6, 3.1, "base"));
    g.stop();
    assert(g.sample(3., 10.5).vx == 0.);
    assert(!g.receive(.1, 0., 0., 10.6, 10.6, 3.1, "base"));
    CommandGate paused;
    paused.setMode(Mode::Autonomy, 20.);
    assert(paused.receive(.1, 0., 0., 20.1, 20.1, 4., "base"));
    // An old queued command on resume is rejected by source age.
    assert(!paused.receive(.1, 0., 0., 20.2, 21., 5., "base"));
    assert(paused.sample(5., 21.).vx == 0.);
    CommandGate modes;
    modes.request(Request::Autonomy);
    assert(modes.takeRequest(false) == Request::None);
    modes.request(Request::Stand);
    assert(modes.mode() == Mode::Passive); // callback cannot change control mode
    assert(modes.takeRequest(false) == Request::Stand);
    modes.setMode(Mode::Stand, 1.);
    modes.request(Request::Autonomy);
    assert(modes.takeRequest(false) == Request::None);
    modes.request(Request::Autonomy);
    assert(modes.takeRequest(true) == Request::Autonomy);
    modes.request(Request::Stand);
    modes.setMode(Mode::Passive, 2.); // keyboard override cancels queued request
    assert(modes.takeRequest(true) == Request::None);
    const Mode actualModes[] = {Mode::Passive, Mode::Manual, Mode::Stand, Mode::Autonomy};
    const Request pendingRequests[] = {Request::None, Request::Stand, Request::Autonomy};
    for (Mode actual : actualModes) {
        for (Request pending : pendingRequests) {
            CommandGate cancel;
            cancel.setMode(actual, 3.);
            if (actual == Mode::Autonomy)
                assert(cancel.receive(.1, 0., 0., 3.1, 3.1, 1., "base"));
            cancel.request(pending);
            cancel.cancelAutonomy();
            assert(cancel.mode() == actual);
            assert(cancel.sample(1., 3.1).vx == 0.);
            assert(cancel.takeRequest(true) == (actual == Mode::Autonomy ? Request::Stand : Request::None));
        }
    }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = pathlib.Path(directory) / "gate.cpp"
            binary = pathlib.Path(directory) / "gate"
            cpp.write_text(source)
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / ".ros1/src/unitree_guide/include"), str(cpp), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

class ReproducibleGuidePatchTests(unittest.TestCase):
    def test_existing_workspace_is_patched_idempotently_without_losing_annotations(self):
        import os
        import shutil
        with tempfile.TemporaryDirectory() as directory:
            fixture = pathlib.Path(directory)
            (fixture / "scripts").mkdir()
            shutil.copy(ROOT / "scripts/ros1_build.sh", fixture / "scripts")
            (fixture / "scripts/compute_guard.sh").write_text("compute_guard_path() { echo /tmp/test-guide-guard; }\nacquire_compute_guard() { :; }\n")
            (fixture / "ros1/patches").mkdir(parents=True)
            shutil.copy(ROOT / "ros1/patches/guide_autonomy.patch", fixture / "ros1/patches")
            source = fixture / ".ros1/src"
            source.mkdir(parents=True)
            shutil.copytree(ROOT / "references/unitree_guide/unitree_guide", source / "unitree_guide")
            for patch in ("ros1/aliengo-guide.patch", "ros1/patches/guide_sim_clock.patch"):
                subprocess.run(["patch", "--batch", "-d", str(source), "-p0", "-i", str(ROOT / patch)], check=True, capture_output=True)
            annotated = source / "unitree_guide/src/FSM/FSM.cpp"
            annotated.write_text("// user annotation preserved\n" + annotated.read_text())
            untouched = source / "unitree_guide/user_notes.txt"
            untouched.write_text("custom source notes\n")
            for plugin in ("livox_laser_simulation", "realsense_gazebo_plugin"):
                (source / plugin).mkdir()
            (fixture / "bin").mkdir()
            docker = fixture / "bin/docker"
            docker.write_text("#!/bin/sh\nexit 0\n")
            docker.chmod(0o755)
            env = dict(os.environ, PATH=str(fixture / "bin") + os.pathsep + os.environ["PATH"])
            for _ in range(2):
                result = subprocess.run(["bash", str(fixture / "scripts/ros1_build.sh")], env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertTrue((source / "unitree_guide/include/autonomy/CommandGate.h").exists())
                self.assertTrue(annotated.read_text().startswith("// user annotation preserved\n"))
                self.assertEqual(untouched.read_text(), "custom source notes\n")

class GuideRosInterfaceTests(unittest.TestCase):
    def test_services_commands_status_and_pause_use_real_ros_interface(self):
        source = r'''
#include "autonomy/GuideAutonomy.h"
#include <cassert>
#include <cmath>
#include <unistd.h>
using namespace guide_autonomy;
std::string status;
void receiveStatus(const std_msgs::String::ConstPtr& message) { status=message->data; }
int main(int argc,char**argv) {
    ros::init(argc,argv,"guide_autonomy_test");
    ros::NodeHandle node;
    ros::Time::setNow(ros::Time(10.));
    GuideAutonomy interface;
    auto stand=node.serviceClient<std_srvs::Trigger>("/aliengo/controller/stand");
    auto autonomy=node.serviceClient<std_srvs::SetBool>("/aliengo/controller/autonomy");
    auto publisher=node.advertise<geometry_msgs::TwistStamped>("/exploration/cmd_vel",1);
    auto subscriber=node.subscribe("/aliengo/controller/status",1,receiveStatus);
    std_srvs::SetBool enable;
    enable.request.data=true;
    assert(autonomy.call(enable) && !enable.response.success);
    std_srvs::Trigger standCall;
    assert(stand.call(standCall) && standCall.response.success);
    assert(interface.takeRequest(false)==Request::Stand);
    interface.transition(Mode::Stand);
    assert(autonomy.call(enable) && !enable.response.success);
    interface.feedback(true);
    assert(autonomy.call(enable) && enable.response.success);
    assert(interface.takeRequest(true)==Request::Autonomy);
    interface.transition(Mode::Autonomy);
    for(int i=0; i<100 && publisher.getNumSubscribers()==0; ++i) usleep(10000);
    assert(publisher.getNumSubscribers()>0);
    ros::Time::setNow(ros::Time(10.1));
    geometry_msgs::TwistStamped command;
    command.header.stamp=ros::Time::now(); command.header.frame_id="base";
    command.twist.linear.x=2.; command.twist.linear.y=-2.; command.twist.angular.z=2.;
    publisher.publish(command); usleep(100000);
    assert(std::abs(interface.sample().vx-.12)<1e-9);
    assert(std::abs(interface.sample().vy+.08)<1e-9);
    assert(std::abs(interface.sample().wz-.30)<1e-9);
    usleep(450000); // no /clock advancement; watchdog must still expire
    assert(interface.sample().vx==0.);
    publisher.publish(command); usleep(50000);
    assert(interface.sample().vx==0.); // frozen source stamp never re-arms
    ros::Time::setNow(ros::Time(10.2)); command.header.stamp=ros::Time::now();
    command.twist.linear.z=NAN; publisher.publish(command); usleep(50000);
    assert(interface.sample().vx==0.);
    ros::Time::setNow(ros::Time(10.3)); command.header.stamp=ros::Time::now();
    command.twist.linear.z=0.; publisher.publish(command); usleep(50000);
    assert(interface.sample().vx>0.);
    enable.request.data=false;
    assert(autonomy.call(enable) && enable.response.success);
    assert(interface.sample().vx==0.); // queued stop clears before control loop consumes it
    assert(interface.takeRequest(false)==Request::Stand);
    interface.transition(Mode::Stand);
    for(int i=0;i<20;++i) { usleep(10000); ros::spinOnce(); }
    assert(status.find("\"mode\":\"stand\"")!=std::string::npos);
    assert(status.find("\"command_valid\":false")!=std::string::npos);
    assert(status.find("\"stand_ready\":false")!=std::string::npos);
    // Late disable RPCs must never undo a keyboard Passive/Manual override.
    const Mode retainedModes[] = {Mode::Passive, Mode::Manual, Mode::Stand};
    for (Mode actual : retainedModes) {
        interface.transition(actual);
        assert(stand.call(standCall) && standCall.response.success);
        enable.request.data=false;
        assert(autonomy.call(enable) && enable.response.success);
        assert(interface.takeRequest(true)==Request::None);
        for(int i=0;i<10;++i) { usleep(10000); ros::spinOnce(); }
        std::string expected=std::string("\"mode\":\"")+CommandGate::name(actual)+"\"";
        assert(status.find(expected)!=std::string::npos);
    }
    interface.transition(Mode::Stand);
    interface.feedback(true);
    enable.request.data=true;
    assert(autonomy.call(enable) && enable.response.success);
    enable.request.data=false;
    assert(autonomy.call(enable) && enable.response.success);
    assert(interface.takeRequest(true)==Request::None); // cancel pending enable
    ros::shutdown();
}
'''
        shell = r'''
set -e
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=127.0.0.1
roscore >/tmp/roscore.log 2>&1 & master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true' EXIT
for attempt in $(seq 1 100); do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep .05
done
g++ -std=c++11 -pthread -DCOMPILE_WITH_SIMULATION -DCOMPILE_WITH_MOVE_BASE \
    -I/work/src/unitree_guide/include -I/opt/ros/noetic/include \
    /work/src/unitree_guide/src/autonomy/GuideAutonomy.cpp -x c++ - \
    -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lcpp_common \
    -lroscpp_serialization -o /tmp/guide-autonomy-test
/tmp/guide-autonomy-test
'''
        result = subprocess.run([
            "docker", "run", "--rm", "-i", "--network", "none", "--memory=512m",
            "--memory-swap=512m", "--cpus=1", "--pids-limit=96",
            "--mount", f"type=bind,source={ROOT / '.ros1'},target=/work,readonly",
            "--entrypoint", "bash", "aliengo-learning:noetic", "-c", shell,
        ], input=source, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
