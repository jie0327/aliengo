#!/usr/bin/env python3
"""Real Gazebo RGB-D geometry regression; invoke with d435_trial.sh."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time
import traceback

import numpy as np
import rosgraph
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState, SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf.transformations import quaternion_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/result'))
    parser.add_argument('--startup-timeout', type=float, default=90)
    parser.add_argument('--camera-timeout', type=float, default=20)
    parser.add_argument('--color-width', type=int, default=1280)
    parser.add_argument('--color-height', type=int, default=720)
    parser.add_argument('--depth-width', type=int, default=848)
    parser.add_argument('--depth-height', type=int, default=480)
    parser.add_argument('--camera-x', type=float, default=.32)
    parser.add_argument('--camera-y', type=float, default=0)
    parser.add_argument('--camera-z', type=float, default=-.03)
    parser.add_argument('--camera-yaw', type=float, default=0)
    args = parser.parse_args()
    values = (args.startup_timeout, args.camera_timeout, args.camera_x,
              args.camera_y, args.camera_z, args.camera_yaw)
    if not all(math.isfinite(value) for value in values):
        parser.error('timeouts and camera pose must be finite')
    dimensions = (args.color_width, args.color_height, args.depth_width, args.depth_height)
    if min(args.startup_timeout, args.camera_timeout) <= 0 or min(dimensions) < 2:
        parser.error('timeouts must be positive and image dimensions at least 2')
    if abs(args.camera_yaw) > .5 or args.camera_x >= 2.2:
        parser.error('front-wall trial requires |camera yaw| <= .5 and camera x < 2.2')
    args.output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    report = {'result': 'FAIL', 'configuration': vars(args).copy()}
    report['configuration']['output'] = str(args.output)
    launch, subscribers, samples = None, [], {}
    lock = threading.Lock()
    phase = 'startup'
    topics = {'color': ('/camera/color/image_raw', Image),
              'color_info': ('/camera/color/camera_info', CameraInfo),
              'depth': ('/camera/depth/image_rect_raw', Image),
              'depth_info': ('/camera/depth/camera_info', CameraInfo),
              'cloud': ('/camera/depth/color/points', PointCloud2),
              'lidar': ('/livox/lidar', PointCloud2), 'imu': ('/livox/imu', Imu)}

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal %s' % signum)

    def timed_out(signum, frame):
        raise TimeoutError('wall timeout in phase ' + phase)

    def record(message, key):
        with lock:
            rows = samples.setdefault(key, [])
            rows.append(message)
            del rows[:-8]

    def wait_for(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError('roslaunch exited: %s' % launch.returncode)
            if predicate():
                return
            time.sleep(.05)
        raise AssertionError('wall timeout waiting for ' + description)

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGALRM, timed_out)
    launch_log = (args.output / 'roslaunch.log').open('w')
    try:
        signal.setitimer(signal.ITIMER_REAL, args.startup_timeout)
        command = ['roslaunch', '/course/ros1/mid360/gazebo.launch', 'rname:=aliengo',
                   'gui:=false', 'paused:=false', 'user_debug:=true', 'spawn_z:=0.6',
                   'sim_ground_truth_tf:=false']
        # Omit unchanged camera args so this also tests the old camera-free launch.
        for name, value, default in (('color_width', args.color_width, 1280),
                                     ('color_height', args.color_height, 720),
                                     ('depth_width', args.depth_width, 848),
                                     ('depth_height', args.depth_height, 480),
                                     ('x', args.camera_x, .32), ('y', args.camera_y, 0),
                                     ('z', args.camera_z, -.03), ('yaw', args.camera_yaw, 0)):
            if value != default:
                command.append('d435_%s:=%s' % (name, value))
        report['command'] = command
        launch = subprocess.Popen(command, stdout=launch_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        deadline = time.monotonic() + args.startup_timeout
        wait_for(rosgraph.is_master_online, args.startup_timeout, 'ROS master')

        def simulation_configured():
            try:
                return rosgraph.Master('/d435_trial_bootstrap').getParam('/use_sim_time') is True
            except (rosgraph.MasterException, OSError):
                return False

        # rospy chooses its clock once at init; master startup precedes launch params.
        wait_for(simulation_configured, max(.1, deadline - time.monotonic()), '/use_sim_time=true')
        rospy.init_node('d435_regression_trial', disable_signals=True)
        wait_for(lambda: rospy.Time.now().to_sec() > 0,
                 max(.1, deadline - time.monotonic()), 'nonzero simulated clock')
        assert not rospy.rostime.is_wallclock(), 'rospy did not select simulated time'
        report['clock'] = {'use_sim_time': True, 'initial_sim_stamp': rospy.Time.now().to_sec()}
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        subscribers = [rospy.Subscriber(topic, message_type, record, callback_args=key,
                                       queue_size=2, buff_size=16 * 1024 * 1024)
                       for key, (topic, message_type) in topics.items()]
        for service in ('/gazebo/spawn_sdf_model', '/gazebo/get_model_state'):
            rospy.wait_for_service(service, timeout=max(.1, deadline - time.monotonic()))
        model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        wait_for(lambda: model('aliengo_gazebo', 'world').success,
                 max(.1, deadline - time.monotonic()), 'AlienGo model')
        phase = 'target'
        pose = Pose()
        pose.position.x, pose.position.z, pose.orientation.w = 2.37, 1., 1.
        axis_y = args.camera_y + (2.31 - args.camera_x) * math.tan(args.camera_yaw)
        patch_centers = [axis_y + .55, axis_y - .55]
        patches = ''.join('''<link name="patch_%s"><pose>-.06 %s 0 0 0 0</pose>
          <collision name="patch"><geometry><box><size>.01 .18 2</size></box></geometry></collision>
          <visual name="patch"><geometry><box><size>.01 .18 2</size></box></geometry>
          <material><ambient>%s 1</ambient><diffuse>%s 1</diffuse></material></visual></link>'''
                          % (index, y, rgb, rgb) for index, (y, rgb) in enumerate(
                              zip(patch_centers, ('1 .01 .01', '.01 1 .01'))))
        target = '''<sdf version="1.6"><model name="d435_target"><static>true</static>
          <link name="wall"><collision name="wall"><geometry><box><size>.1 6 4</size>
          </box></geometry></collision><visual name="wall"><geometry><box><size>.1 6 4</size>
          </box></geometry><material><ambient>.01 .01 .01 1</ambient><diffuse>.01 .01 .01 1</diffuse>
          </material></visual></link>''' + patches + '</model></sdf>'
        spawned = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
            'd435_target', target, '', pose, 'world')
        assert spawned.success, spawned.status_message
        time.sleep(.3)
        target_stamp = rospy.Time.now().to_sec()
        report['target'] = {'front_x_m': 2.32, 'patch_front_x_m': 2.305,
                            'patch_y_m': patch_centers, 'after_stamp': target_stamp}
        with lock:
            samples.clear()
        phase = 'streams'
        signal.setitimer(signal.ITIMER_REAL, args.camera_timeout)

        def ready():
            with lock:
                return all(len({m.header.stamp.to_sec() for m in samples.get(key, [])
                                if m.header.stamp.to_sec() > target_stamp}) >= 2 for key in topics)

        wait_for(ready, args.camera_timeout, 'two increasing post-target frames on all seven topics')
        with lock:
            captured = {key: list(rows) for key, rows in samples.items()}
        phase = 'geometry'
        signal.setitimer(signal.ITIMER_REAL, 15)
        frames = {}
        for key, rows in captured.items():
            stamps = [m.header.stamp.to_sec() for m in rows if m.header.stamp.to_sec() > target_stamp]
            assert len(stamps) >= 2 and stamps[-1] > stamps[0], key + ' stamps did not increase'
            frames[key] = rows[-1]
        report['streams'] = {key: {'topic': topics[key][0], 'frames': len(rows),
                                  'first_stamp': rows[0].header.stamp.to_sec(),
                                  'last_stamp': rows[-1].header.stamp.to_sec(),
                                  'frame_id': rows[-1].header.frame_id} for key, rows in captured.items()}
        for key in ('color', 'depth', 'color_info', 'depth_info', 'cloud'):
            message = frames[key]
            stream = 'color' if key in ('color', 'color_info') else 'depth'
            dimensions = ((args.color_width, args.color_height) if stream == 'color'
                          else (args.depth_width, args.depth_height))
            assert (message.width, message.height) == dimensions, key + ' dimensions'
            assert message.header.frame_id == 'd435_%s_optical_frame' % stream, key + ' frame: ' + message.header.frame_id
        color, depth, cloud = frames['color'], frames['depth'], frames['cloud']
        report['encodings'] = {'color': color.encoding, 'depth': depth.encoding}
        assert color.encoding == 'rgb8', 'RGB encoding: ' + color.encoding
        assert depth.encoding == '16UC1', 'expected 16UC1 millimeter depth, received ' + depth.encoding
        assert len(color.data) >= color.step * color.height, 'truncated RGB image'
        assert len(depth.data) >= depth.step * depth.height, 'truncated depth image'
        assert len(cloud.data) >= cloud.row_step * cloud.height, 'truncated organized cloud'
        rgb_bytes = b''.join(bytes(color.data[y * color.step:y * color.step + color.width * 3])
                             for y in range(color.height))
        (args.output / 'color.ppm').write_bytes(
            ('P6\n%s %s\n255\n' % (color.width, color.height)).encode() + rgb_bytes)
        with (args.output / 'color.ppm').open('wb') as preview:
            preview.write(('P6\n%s %s\n255\n' % (color.width, color.height)).encode('ascii'))
            for row_index in range(color.height):
                start = row_index * color.step
                preview.write(color.data[start:start + color.width * 3])
        report['intrinsics'] = {}
        for key, fov in (('color_info', 69), ('depth_info', 87)):
            info = frames[key]
            assert info.K[0] > 0 and info.K[4] > 0 and info.K[8] == 1, key + ' invalid focal length'
            assert abs(info.K[2] - info.width / 2) < 2 and abs(info.K[5] - info.height / 2) < 2, key + ' principal point'
            assert all(math.isfinite(v) for v in list(info.K) + list(info.P)), key + ' nonfinite intrinsics'
            measured_fov = math.degrees(2 * math.atan(info.width / (2 * info.K[0])))
            assert abs(measured_fov - fov) < 1, key + ' horizontal FOV: %s' % measured_fov
            report['intrinsics'][key] = {'K': list(info.K), 'horizontal_fov_degrees': measured_fov}
        u, v = args.depth_width // 2, args.depth_height // 2
        pixels = [(x, y) for y in range(max(0, v - 4), min(args.depth_height, v + 5))
                  for x in range(max(0, u - 4), min(args.depth_width, u + 5))]
        depths = [struct.unpack_from(('>' if depth.is_bigendian else '<') + 'H',
                                    depth.data, y * depth.step + x * 2)[0] * .001 for x, y in pixels]
        valid = [d for d in depths if math.isfinite(d) and d > 0]
        assert len(valid) >= len(pixels) * .9, 'insufficient finite positive central metric depth'
        expected = (2.32 - args.camera_x) / math.cos(args.camera_yaw)
        measured = float(np.median(valid))
        report['depth_m'] = {'expected': expected, 'central_median': measured, 'finite_samples': len(valid)}
        assert abs(measured - expected) < .12, 'wall depth mismatch: %s' % report['depth_m']
        points = list(point_cloud2.read_points(cloud, ('x', 'y', 'z'), skip_nans=True, uvs=pixels))
        points = [p for p in points if all(math.isfinite(c) for c in p)]
        assert len(points) >= len(pixels) * .9, 'insufficient finite central XYZ cloud points'
        center = np.median(points, axis=0)
        report['cloud'] = {'width': cloud.width, 'height': cloud.height, 'finite_samples': len(points),
                           'central_xyz_m': center.tolist(), 'fields': [f.name for f in cloud.fields]}
        assert abs(center[2] - measured) < .04 and abs(center[0]) < .06 and abs(center[1]) < .06, 'cloud optical geometry mismatch'
        rgb_field = next((f for f in cloud.fields if f.name in ('rgb', 'rgba')), None)
        assert rgb_field is not None, 'cloud RGB field missing'
        assert frames['lidar'].width * frames['lidar'].height > 0, 'MID360 cloud empty'
        stamp = depth.header.stamp
        expected_rotation = np.array([[math.sin(args.camera_yaw), 0, math.cos(args.camera_yaw)],
                                      [-math.cos(args.camera_yaw), 0, math.sin(args.camera_yaw)],
                                      [0, -1, 0]])
        report['optical_tf'] = {}
        for parent in ('base', 'trunk'):
            for stream in ('depth', 'color'):
                transform = tf_buffer.lookup_transform(parent, 'd435_%s_optical_frame' % stream,
                                                       stamp, rospy.Duration(0)).transform
                t, q = transform.translation, transform.rotation
                xyz = [t.x, t.y, t.z]
                rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
                offset = .015 if stream == 'color' else 0
                expected_xyz = [args.camera_x - offset * math.sin(args.camera_yaw),
                                args.camera_y + offset * math.cos(args.camera_yaw), args.camera_z]
                report['optical_tf'][parent + '_to_' + stream] = {'xyz': xyz, 'rotation': rotation.tolist()}
                assert np.allclose(xyz, expected_xyz, atol=.002), parent + ' ' + stream + ' camera translation'
                assert np.allclose(rotation, expected_rotation, atol=.002), parent + ' ' + stream + ' optical axes'

        # Project measured depth XYZ through the independent, published color optics.
        transform = tf_buffer.lookup_transform('d435_color_optical_frame', 'd435_depth_optical_frame',
                                               cloud.header.stamp, rospy.Duration(0)).transform
        t, q = transform.translation, transform.rotation
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        translation = np.array([t.x, t.y, t.z])
        color_k, depth_k = frames['color_info'].K, frames['depth_info'].K

        def image_rgb(x, y):
            offset = y * color.step + x * 3
            return np.array(list(color.data[offset:offset + 3]), dtype=float)

        def cloud_rgb(x, y):
            offset = y * cloud.row_step + x * cloud.point_step + rgb_field.offset
            packed = struct.unpack_from(('>' if cloud.is_bigendian else '<') + 'I', cloud.data, offset)[0]
            return np.array([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], dtype=float)

        row = list(point_cloud2.read_points(cloud, ('x', 'y', 'z'), skip_nans=False,
                                           uvs=[(x, v) for x in range(cloud.width)]))
        (args.output / 'color_projection_row.json').write_text(json.dumps(
            {'depth_v': v, 'color_K': list(color_k), 'depth_K': list(depth_k),
             'depth_to_color_translation': translation.tolist(),
             'depth_to_color_rotation': rotation.tolist(),
             'points': [{'depth_u': x, 'xyz': list(point), 'rgb': cloud_rgb(x, v).tolist()}
                        for x, point in enumerate(row) if all(math.isfinite(value) for value in point)]},
            indent=2) + '\n')
        projections, outside = {}, []
        for x, point in enumerate(row):
            if not all(math.isfinite(value) for value in point) or point[2] <= 0:
                continue
            cp = rotation.dot(point) + translation
            if cp[2] <= 0:
                continue
            cx = int(round(color_k[0] * cp[0] / cp[2] + color_k[2]))
            cy = int(round(color_k[4] * cp[1] / cp[2] + color_k[5]))
            if cx < -2 or cx >= color.width + 2 or cy < -2 or cy >= color.height + 2:
                outside.append((x, cloud_rgb(x, v)))
            elif 2 <= cx < color.width - 2 and 2 <= cy < color.height - 2:
                projections[x] = (cx, cy, image_rgb(cx, cy), cloud_rgb(x, v))
        assert len(outside) >= 5, 'no finite depth geometry sampled beyond narrower RGB FOV'
        assert all(np.max(rgb) == 0 for _, rgb in outside), 'out-of-color-FOV cloud points are not black'
        report['cloud']['finite_points_outside_color_fov'] = len(outside)
        report['color_projection'] = []
        for index, patch_y in enumerate(patch_centers):
            dx, dy = 2.305 - args.camera_x, patch_y - args.camera_y
            optical_x = math.sin(args.camera_yaw) * dx - math.cos(args.camera_yaw) * dy
            optical_z = math.cos(args.camera_yaw) * dx + math.sin(args.camera_yaw) * dy
            patch_u = int(round(depth_k[0] * optical_x / optical_z + depth_k[2]))
            assert patch_u in projections, 'known colored patch outside color/depth overlap'
            cx, cy, expected_rgb, actual_rgb = projections[patch_u]
            patch_report = {'patch': ('red', 'green')[index], 'depth_u': patch_u,
                            'center_rgb': actual_rgb.tolist(), 'baseline_sensitive_edges': []}
            report['color_projection'].append(patch_report)
            assert expected_rgb[index] > max(np.delete(expected_rgb, index)) + 60, 'known red/green patch not visible in RGB'
            assert np.max(np.abs(actual_rgb - expected_rgb)) <= 35, 'patch cloud RGB disagrees with independent color projection'
            # Select edge rays where the 15 mm baseline changes the observed color.
            # This detects a forgotten translation as well as same-pixel/IR coloring.
            edge_checks = []
            radius = depth_k[0] * .18 / optical_z + 3
            for x, (cx, cy, expected_rgb, actual_rgb) in projections.items():
                if abs(x - patch_u) > radius:
                    continue
                wrong_cx = int(round(color_k[0] * row[x][0] / row[x][2] + color_k[2]))
                if not 0 <= wrong_cx < color.width:
                    continue
                neighborhood = [image_rgb(cx + shift, cy) for shift in (-1, 0, 1)]
                if (max(np.max(np.abs(rgb - expected_rgb)) for rgb in neighborhood) < 15
                        and np.max(np.abs(image_rgb(wrong_cx, cy) - expected_rgb)) > 80):
                    edge_checks.append({'depth_u': x, 'color_uv': [cx, cy],
                                        'no_baseline_rgb_difference': float(np.max(np.abs(image_rgb(wrong_cx, cy) - expected_rgb))),
                                        'rgb_error': float(np.max(np.abs(actual_rgb - expected_rgb)))})
            patch_report['baseline_sensitive_edges'] = edge_checks
            assert edge_checks, 'no baseline-sensitive color boundary samples for patch %s' % index
            assert max(check['rgb_error'] for check in edge_checks) <= 35, 'color baseline projection mismatch: %s' % edge_checks
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = '%s: %s' % (type(error).__name__, error)
        report['traceback'] = traceback.format_exc()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if launch is not None:
            try:
                os.killpg(launch.pid, signal.SIGINT)
                launch.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                pass
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except OSError:
                pass
            try:
                launch.wait(timeout=3)
            except subprocess.TimeoutExpired:
                report['cleanup_error'] = 'roslaunch did not exit after SIGKILL'
                report['result'] = 'FAIL'
        if rospy.core.is_initialized():
            rospy.signal_shutdown('trial complete')
        with lock:
            report['observed_frames'] = {key: len(samples.get(key, [])) for key in topics}
        report.update(phase=phase, wall_seconds=time.monotonic() - started)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        launch_log.close()
    print('D435 trial:', report['result'], report.get('error', ''), flush=True)
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
