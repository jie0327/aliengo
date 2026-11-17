#!/usr/bin/env python3
"""Actual vendor UDP transport and real Livox LIO branch, in network-none Docker."""
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped, TwistStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, CameraInfo
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, SetBool

sys.path.insert(0, '/course/scripts')
from hardware_config import planner_sensor

OUT = Path('/hardware/test-output')
OUT.mkdir(exist_ok=True)
processes, streams = [], []


def spawn(command, name):
    stream = open(OUT/(name+'.log'), 'w')
    streams.append(stream)
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
    processes.append(process)
    return process


def until(predicate, timeout=8.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return False


def crc(data):
    value = 0xffffffff
    for word, in struct.iter_unpack('<I', data):
        for bit in range(32):
            value = ((value << 1) ^ (0x04c11db7 if bool(value & 0x80000000) ^
                     bool(word & (1 << (31-bit))) else 0)) & 0xffffffff
    return value


class VendorPeer:
    """Hardware boundary double: real SDK still owns serialization/CRC/sockets."""
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(.05)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.remote = False
        self.respond = True
        self.packets = []
        self.states = {}
        for mode in (1, 2):
            for remote in (False, True):
                packet = bytearray(244)
                packet[0], packet[10] = 0, mode
                struct.pack_into('<f', packet, 11, 1.)
                if remote:
                    struct.pack_into('<H', packet, 198, 0x220)  # vendor L2+B
                struct.pack_into('<I', packet, 240, crc(packet[:240]))
                self.states[mode, remote] = packet
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                packet, peer = self.socket.recvfrom(2048)
            except socket.timeout:
                continue
            self.packets.append((time.monotonic(), packet))
            if len(packet) == 113 and self.respond:
                self.socket.sendto(self.states[2 if packet[10] == 2 else 1, self.remote], peer)

    def moving(self):
        return any(len(p) == 113 and abs(struct.unpack_from('<f', p, 41)[0]) > .01
                   for t, p in self.packets[-25:])

    def close(self):
        self.stop.set()
        self.thread.join(1.)
        self.socket.close()


def sdk_check():
    peer = VendorPeer()
    statuses = []
    sub = rospy.Subscriber('/aliengo/controller/status', String,
                           lambda m: statuses.append(json.loads(m.data)), queue_size=50)
    pub = rospy.Publisher('/exploration/cmd_vel', TwistStamped, queue_size=1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as local:
        local.bind(('127.0.0.1', 0))
        local_port = local.getsockname()[1]
    process = spawn(['/hardware/bin/high_level_bridge', '_robot_ip:=127.0.0.1',
                     '_robot_port:='+str(peer.port), '_local_port:='+str(local_port)], 'sdk')
    try:
        assert until(lambda: statuses and statuses[-1]['sdk_feedback_fresh']), 'SDK did not accept loopback HighState CRC'
        assert peer.packets and all(len(p) == 113 for t, p in peer.packets), 'wrong HighCmd ABI'
        assert all(struct.unpack_from('<fff', p, 41) == (0., 0., 0.) for t, p in peer.packets), 'startup moved'
        stand = rospy.ServiceProxy('/aliengo/controller/stand', Trigger)
        mode = rospy.ServiceProxy('/aliengo/controller/autonomy', SetBool)
        assert stand().success
        assert until(lambda: statuses[-1]['stand_ready'])
        assert mode(True).success
        assert until(lambda: pub.get_num_connections())

        def command_for(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                msg = TwistStamped()
                msg.header.stamp, msg.header.frame_id = rospy.Time.now(), 'base'
                msg.twist.linear.x, msg.twist.angular.z = .1, .2
                pub.publish(msg)
                time.sleep(.03)
        command_for(.25)
        assert peer.moving(), 'fresh body velocity did not reach vendor UDP'
        time.sleep(.55)
        assert not peer.moving() and statuses[-1]['mode'] == 'stand', 'command timeout failed'
        assert stand().success
        assert until(lambda: statuses[-1]['stand_ready'])
        assert mode(True).success
        command_for(.2)
        peer.respond = False
        command_for(.6)
        assert not peer.moving() and not statuses[-1]['sdk_feedback_fresh'], 'feedback loss did not stop'
        peer.respond = True
        assert until(lambda: statuses[-1]['sdk_feedback_fresh'])
        assert statuses[-1]['mode'] == 'stand', 'feedback resumed autonomy'
        assert stand().success
        assert until(lambda: statuses[-1]['stand_ready'])
        assert mode(True).success
        command_for(.2)
        peer.remote = True
        assert until(lambda: not statuses[-1]['sending']), 'remote takeover did not release UDP'
        count = len(peer.packets)
        time.sleep(.2)
        assert len(peer.packets) == count, 'commands continued after handheld takeover'
        assert mode(False).success
        print('SDK transport: ABI, startup zero, velocity, command loss, feedback loss, takeover PASS', flush=True)
    finally:
        process.terminate()
        process.wait(timeout=5)
        sub.unregister()
        pub.unregister()
        peer.close()


def lio_check():
    peer = VendorPeer()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as local:
        local.bind(('127.0.0.1', 0))
        local_port = local.getsockname()[1]
    sdk_process = spawn(['/hardware/bin/high_level_bridge', '_robot_ip:=127.0.0.1',
                         '_robot_port:='+str(peer.port), '_local_port:='+str(local_port)], 'sdk-pipeline')
    map_started = time.time()
    config = dict(base_to_mid360=dict(xyz=[.25, 0., .145], rpy=[0., 0., 0.]),
                  base_to_d435=dict(xyz=[.32, 0., -.03], rpy=[0., 0., 0.]),
                  imu_acceleration_scale=9.80665)
    Path('/tmp/hardware-test.yaml').write_text(yaml.safe_dump(config))
    Path('/tmp/hardware-test-sensor.yaml').write_text(yaml.safe_dump(planner_sensor(config)))
    process = spawn(['roslaunch', '/course/ros1/hardware/real.launch',
                     'config:=/tmp/hardware-test.yaml', 'sensor_config:=/tmp/hardware-test-sensor.yaml',
                     'map_dir:=../../../hardware/test-output/maps'], 'pipeline')
    states, odometry, clouds, scans, imus = [], [], [], [], []
    subscribers = [
        rospy.Subscriber('/exploration/terrain_status', String, lambda m: states.append(json.loads(m.data))),
        rospy.Subscriber('/lio/odom', Odometry, odometry.append),
        rospy.Subscriber('/livox/lidar', PointCloud2, clouds.append),
        rospy.Subscriber('/livox/lidar_custom', CustomMsg, scans.append),
        rospy.Subscriber('/livox/imu', Imu, imus.append)]
    lidar = rospy.Publisher('/livox/raw_lidar', CustomMsg, queue_size=2)
    imu = rospy.Publisher('/livox/raw_imu', Imu, queue_size=50)
    camera = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=2)
    camera_info = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=2)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp, transform.header.frame_id = rospy.Time.now(), 'd435_link'
    transform.child_frame_id = 'd435_depth_optical_frame'
    transform.transform.rotation.x, transform.transform.rotation.y = -.5, .5
    transform.transform.rotation.z, transform.transform.rotation.w = -.5, .5
    broadcaster.sendTransform(transform)
    points = []
    for i in range(-20, 21):
        for j in range(-20, 21):
            x, y = i*.15, j*.15
            if abs(x) > 1 or abs(y) > 1:
                points.append((x, y, -.575))
            points.append((x, 3., j*.1))
            points.append((3., y, j*.1))
    cp = [CustomPoint(offset_time=int(k*99000000/max(1,len(points)-1)),
                      x=p[0], y=p[1], z=p[2], reflectivity=30, tag=0x10, line=0)
          for k, p in enumerate(points)]
    cp.insert(0, CustomPoint(offset_time=0, x=.4, y=0., z=0., reflectivity=30, tag=0x10, line=0))
    last_scan = 0.
    checker = None
    try:
        deadline = time.monotonic()+18.
        while time.monotonic() < deadline:
            stamp = rospy.Time.now()
            msg = Imu()
            msg.header = Header(stamp=stamp, frame_id='livox_frame')
            msg.linear_acceleration.z = 1.
            imu.publish(msg)
            if time.monotonic()-last_scan > .1:
                start = stamp-rospy.Duration(.1)
                raw = CustomMsg(header=Header(stamp=start, frame_id='mid360_link'),
                                timebase=start.to_nsec(), point_num=len(cp), lidar_id=0,
                                rsvd=[0, 0, 0], points=cp)
                lidar.publish(raw)
                image = Image()
                image.header = Header(stamp=stamp, frame_id='d435_depth_optical_frame')
                image.width, image.height, image.step, image.encoding = 32, 24, 64, '16UC1'
                # Lower rows view a real plane at optical y=.4m; upper rows a wall.
                image.data = b''.join(struct.pack('<H', int(1000*(.4*20/(v-11.5) if v>13 else 5.)))
                                      for v in range(24) for u in range(32))
                info = CameraInfo(header=image.header, width=32, height=24,
                                  distortion_model='plumb_bob', D=[0.]*5,
                                  K=[20.,0.,15.5,0.,20.,11.5,0.,0.,1.],
                                  R=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                                  P=[20.,0.,15.5,0.,0.,20.,11.5,0.,0.,0.,1.,0.])
                camera_info.publish(info)
                camera.publish(image)
                last_scan = time.monotonic()
            if states and clouds and odometry and all(
                    states[-1]['sources'][s]['count'] > 3 for s in ('lidar', 'camera')):
                if checker is None:
                    checker = spawn(['python3', '/course/ros1/hardware/scripts/readiness.py', '--motion'], 'readiness')
                elif checker.poll() is not None:
                    assert checker.returncode == 0, 'read-only readiness check failed'
                    break
            assert process.poll() is None, 'real pipeline exited'
            time.sleep(.005)
        assert odometry and clouds, 'real CustomMsg did not produce LIO odometry/deskewed cloud'
        assert states and all(states[-1]['sources'][s]['count'] > 3 for s in ('lidar', 'camera')), 'fusion did not process both real-time streams'
        assert not rospy.get_param('/use_sim_time'), 'real launch enabled simulation clock'
        assert rospy.get_param('/lio/sensor/lidar_type') == 1
        assert scans[-1].points[-1].offset_time == 99000000
        assert scans[-1].timebase == scans[-1].header.stamp.to_nsec()
        assert imus[-1].header.frame_id == 'mid360_imu_link'
        assert abs(imus[-1].linear_acceleration.z-9.80665) < 1e-8
        assert clouds[-1].header.frame_id == 'mid360_link'
        assert clouds[-1].width > 2000, 'fusion lost fine returns to the LIO registration voxel'
        from sensor_msgs import point_cloud2
        assert any(abs(x-.4)<.01 and abs(y)<.01 and abs(z)<.01 for x,y,z in
                   point_cloud2.read_points(clouds[-1], field_names=('x','y','z'), skip_nans=True)), \
            'near pillar return was lost to the LIO registration blind distance'
        assert checker and checker.poll() == 0, 'readiness command did not complete'
        assert all(math.isfinite(v) for v in (odometry[-1].pose.pose.position.x,
                                              odometry[-1].pose.pose.position.y))
        (OUT/'pipeline-result.json').write_text(json.dumps(dict(
            odometry=len(odometry), deskewed_clouds=len(clouds),
            last_cloud_points=clouds[-1].width, terrain=states[-1]), indent=2))
        print('Real sensor pipeline: Driver2 -> official LIO type=1 -> full deskewed rays; depth image -> official XYZ -> dual fusion PASS', flush=True)
    finally:
        process.send_signal(2)
        process.wait(timeout=20.)
        assert (OUT/'maps/map.pcd').is_file(), 'LIO did not persist the session map'
        assert (OUT/'maps/map.pcd').stat().st_mtime > map_started, 'map is left over from an earlier test'
        sdk_process.terminate()
        sdk_process.wait(timeout=5.)
        peer.close()
        for s in subscribers:
            s.unregister()
        for p in (lidar, imu, camera, camera_info):
            p.unregister()


def main():
    master = spawn(['roscore', '-p', '11325'], 'master')
    rospy.init_node('hardware_integration_check', anonymous=True, disable_signals=True)
    assert until(lambda: master.poll() is None and master_ready()), 'master did not start'
    try:
        sdk_check()
        lio_check()
    finally:
        rospy.signal_shutdown('tests complete')
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()
                p.wait(timeout=10.)
        for s in streams:
            s.close()


def master_ready():
    try:
        return rospy.get_master().getPid()[0] == 1
    except Exception:
        return False


if __name__ == '__main__':
    main()
