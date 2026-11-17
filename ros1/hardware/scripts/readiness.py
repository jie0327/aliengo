#!/usr/bin/env python3
"""Read-only ROS checks; never call stand/autonomy or publish velocity."""
import argparse
import json
import time
import rospy
import tf2_ros
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--motion', action='store_true', help='also require SDK feedback/services')
args = parser.parse_args()
rospy.init_node('hardware_readiness', anonymous=True)
seen, payloads = {}, {}
subscriptions = []
topics = [('/livox/raw_lidar', CustomMsg), ('/livox/imu', Imu),
          ('/camera/depth/color/points', PointCloud2), ('/lio/odom', Odometry),
          ('/exploration/terrain_status', String), ('/exploration/map_status', String),
          ('/exploration/status', String), ('/aliengo/controller/status', String)]
def callback(msg, topic):
    seen.setdefault(topic, []).append(time.monotonic())
    if hasattr(msg, 'data') and isinstance(msg.data, str):
        try:
            payloads[topic] = json.loads(msg.data)
        except ValueError:
            pass
    elif hasattr(msg, 'header'):
        payloads[topic] = dict(stamp=msg.header.stamp.to_sec(), frame=msg.header.frame_id)
        if isinstance(msg, Imu):
            a = msg.linear_acceleration
            payloads[topic]['acceleration_norm'] = (a.x*a.x+a.y*a.y+a.z*a.z)**.5
for topic, kind in topics:
    subscriptions.append(rospy.Subscriber(topic, kind, callback, callback_args=topic, queue_size=5))
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
time.sleep(3.)
failures, report = [], {}
for topic, kind in topics:
    times = seen.get(topic, [])
    optional = topic == '/aliengo/controller/status' and not args.motion
    active = len(times) >= 2 and time.monotonic()-times[-1] < 1.5
    report[topic] = dict(active=active, messages=len(times), latest=payloads.get(topic))
    if not active and not optional:
        failures.append(topic+' missing/stopped')
    latest = payloads.get(topic, {})
    stamp = latest.get('stamp')
    if stamp is not None and not -.05 <= rospy.Time.now().to_sec()-stamp <= 1.5:
        failures.append(topic+' timestamp stale or clock mismatch')
    if topic in ('/exploration/terrain_status', '/exploration/map_status'):
        for name, source in latest.get('sources', {}).items():
            if source.get('count', 0) == 0 or not -.05 <= rospy.Time.now().to_sec()-source.get('stamp', 0) <= 1.5:
                failures.append(topic+' source '+name+' not progressing')
for frame in ('mid360_link', 'mid360_imu_link', 'd435_depth_optical_frame'):
    if not buffer.can_transform('lio_map', frame, rospy.Time(0), rospy.Duration(.1)):
        failures.append('missing TF lio_map -> '+frame)
if rospy.get_param('/use_sim_time', False):
    failures.append('/use_sim_time must be false')
exploration = payloads.get('/exploration/status', {})
if args.motion:
    if not payloads.get('/aliengo/controller/status', {}).get('sdk_feedback_fresh', False):
        failures.append('no fresh vendor HighState')
    services = rospy.get_master().getSystemState()[2][2]
    names = {name for name, providers in services}
    for name in ('/aliengo/controller/stand', '/aliengo/controller/autonomy', '/exploration/enable'):
        if name not in names:
            failures.append('missing '+name)
    if not exploration.get('inputs_ready', False):
        failures.append('coordinator: '+str(exploration.get('reason', 'inputs not ready')))
print(json.dumps(dict(ok=not failures, failures=failures, topics=report,
                      exploration=exploration), ensure_ascii=False, indent=2))
raise SystemExit(1 if failures else 0)
