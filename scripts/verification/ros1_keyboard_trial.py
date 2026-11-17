#!/usr/bin/env python3
"""Bounded Gazebo keyboard trial; run inside the ROS 1 simulation container."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import shutil
import subprocess
import tempfile
import threading
import time

import rosgraph
import rosnode
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from rosgraph_msgs.msg import Clock
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import Empty


def tf_motion_metrics(rows, spawn_yaw, gazebo_progress):
    """Reject a frozen/missing world TF and compare it with each Gazebo sample."""
    assert len(rows) >= 2, 'not enough walk samples to validate world TF motion'
    assert all(row.get('world_tf') is not None for row in rows), 'walk world TF samples missing'
    errors = [math.sqrt((row['world_tf']['x'] - row['x']) ** 2
                        + (row['world_tf']['y'] - row['y']) ** 2
                        + (row['world_tf']['z'] - row['world_z']) ** 2)
              for row in rows]
    first, last = rows[0]['world_tf'], rows[-1]['world_tf']
    progress = ((last['x'] - first['x']) * math.cos(spawn_yaw)
                + (last['y'] - first['y']) * math.sin(spawn_yaw))
    metrics = {'samples': len(rows), 'start_world_tf': first, 'end_world_tf': last,
               'tf_forward_progress': progress, 'gazebo_forward_progress': gazebo_progress,
               'forward_progress_error_m': abs(progress - gazebo_progress),
               'max_position_error_m': max(errors)}
    assert all(math.isfinite(error) for error in errors), 'non-finite world TF position error'
    assert metrics['max_position_error_m'] < .03, 'walk world TF/Gazebo position error: %s' % metrics
    assert metrics['forward_progress_error_m'] < .03, 'world TF forward motion mismatch: %s' % metrics
    return metrics


def rviz_view_metrics(config):
    """Validate the view exported by the running RViz, not the launch template."""
    manager = config.get('Visualization Manager', {})
    current = manager.get('Views', {}).get('Current', {})
    metrics = {'fixed_frame': manager.get('Global Options', {}).get('Fixed Frame'),
               'view_class': current.get('Class'), 'target_frame': current.get('Target Frame')}
    assert metrics['fixed_frame'] == 'world', 'RViz fixed frame is not world: %s' % metrics
    assert metrics['view_class'] == 'rviz/Orbit', 'RViz current view is not Orbit: %s' % metrics
    assert metrics['target_frame'] == '<Fixed Frame>', 'RViz camera follows a moving frame: %s' % metrics
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', choices=('stock', 'mid360'), required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('/result'))
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--downsample', type=int, default=4)
    parser.add_argument('--check-pause', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--world-file')
    parser.add_argument('--spawn-x', type=float, default=0)
    parser.add_argument('--spawn-y', type=float, default=0)
    parser.add_argument('--spawn-z', type=float, default=.6)
    parser.add_argument('--spawn-yaw', type=float, default=0)
    parser.add_argument('--ground-z', type=float, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if not 1 <= args.downsample <= 24000:
        parser.error('--downsample must be between 1 and 24000')
    args.output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    deadline = started + 300
    report = {'result': 'FAIL', 'launch': args.launch, 'seconds': args.seconds,
              'gui': args.gui, 'rviz': args.rviz, 'world_file': args.world_file,
              'ground_z': args.ground_z, 'downsample': args.downsample, 'phases': {}}
    samples, lidar_frames = [], []
    state = {'sim': None, 'pose': None, 'lidar_count': 0, 'imu_count': 0,
             'lidar_points': None, 'walk_lidar_geometry': None,
             'lidar_header': None, 'world_tf': None, 'world_tf_error': None,
             'pose_stamp': None}
    lock = threading.Lock()
    launch = controller = None
    terminal = None
    phase = 'startup'
    check_pose = False
    next_sample = started
    next_pose_query = started
    model_proxy = None
    # Retain both handles throughout the trial so TF subscriptions remain active.
    tf_buffer = tf_listener = None
    launch_log = (args.output / 'launch.log').open('w')
    controller_log = (args.output / 'controller.log').open('wb')

    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal %s' % signum)

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)

    def snapshot():
        with lock:
            return dict(state)

    def on_clock(message):
        with lock:
            state['sim'] = message.clock.to_sec()

    def update_pose(pose):
        q = pose.orientation
        value = {'x': pose.position.x, 'y': pose.position.y,
                 'z': pose.position.z - args.ground_z, 'world_z': pose.position.z,
                 'roll': math.atan2(2 * (q.w*q.x + q.y*q.z),
                                    1 - 2 * (q.x*q.x + q.y*q.y)),
                 'pitch': math.asin(max(-1, min(1, 2 * (q.w*q.y - q.z*q.x)))),
                 'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y),
                                   1 - 2 * (q.y*q.y + q.z*q.z))}
        with lock:
            state['pose'] = value

    def on_lidar(message):
        now = time.monotonic()
        count = message.width * message.height
        with lock:
            state['lidar_count'] += 1
            state['lidar_points'] = count
            state['lidar_header'] = message.header
            lidar_frames.append({'wall': now - started,
                                 'stamp': message.header.stamp.to_sec(), 'points': count})
            if phase == 'walk' and count > 0 and state['walk_lidar_geometry'] is None:
                # Inspect one frame in its original lidar coordinates, without a map transform.
                points = [point for point in point_cloud2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True)
                    if all(math.isfinite(value) for value in point)]
                ranges = {
                    axis + '_range_m': [min(point[i] for point in points),
                                       max(point[i] for point in points)] if points else None
                    for i, axis in enumerate('xyz')}
                state['walk_lidar_geometry'] = {
                    'frame_id': message.header.frame_id,
                    'stamp': message.header.stamp.to_sec(),
                    'points': count, 'finite_xyz_points': len(points),
                    'z_above_1m_points': sum(point[2] > 1.0 for point in points),
                    **ranges}

    def on_imu(message):
        with lock:
            state['imu_count'] += 1

    def tick(force=False):
        nonlocal next_sample, next_pose_query
        now = time.monotonic()
        # 大地图不要订阅每个物理步发布的全场景 ModelStates；只查询机器狗。
        if model_proxy is not None and now >= next_pose_query:
            model = model_proxy('aliengo_gazebo', 'world')
            if model.success:
                update_pose(model.pose)
                world_tf, tf_error = None, None
                if tf_buffer is not None:
                    try:
                        transform = tf_buffer.lookup_transform(
                            'world', 'base', model.header.stamp, rospy.Duration(.2))
                        position = transform.transform.translation
                        world_tf = {'x': position.x, 'y': position.y, 'z': position.z,
                                    'stamp': transform.header.stamp.to_sec()}
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException) as error:
                        tf_error = str(error)
                with lock:
                    state['pose_stamp'] = model.header.stamp.to_sec()
                    state['world_tf'] = world_tf
                    state['world_tf_error'] = tf_error
            next_pose_query = now + .1
        current = snapshot()
        if current['pose'] is not None and current['sim'] is not None:
            pose = current['pose']
            if force or now >= next_sample:
                sample = {'wall': now - started, 'monotonic': now, 'phase': phase,
                          'sim': current['sim'], **pose,
                          'lidar_count': current['lidar_count'],
                          'imu_count': current['imu_count'],
                          'lidar_points': current['lidar_points']}
                if args.launch == 'mid360':
                    sample.update(gazebo_pose_stamp=current['pose_stamp'],
                                  world_tf=current['world_tf'],
                                  world_tf_error=current['world_tf_error'])
                    if current['world_tf'] is not None:
                        position = current['world_tf']
                        sample['world_tf_position_error_m'] = math.sqrt(
                            (position['x'] - pose['x']) ** 2
                            + (position['y'] - pose['y']) ** 2
                            + (position['z'] - pose['world_z']) ** 2)
                previous = samples[-1] if samples else None
                sample['rtf'] = ((sample['sim'] - previous['sim']) /
                                 (sample['wall'] - previous['wall'])
                                 if previous and sample['wall'] > previous['wall'] else None)
                samples.append(sample)
                next_sample = now + .1
            if check_pose and (not all(math.isfinite(v) for v in pose.values()) or
                               pose['z'] < .18 or abs(pose['roll']) > .8 or
                               abs(pose['pitch']) > .8):
                raise RuntimeError('posture threshold exceeded: %s' % pose)
        if now >= deadline:
            raise RuntimeError('300 second wall-clock deadline exceeded')
        if launch is not None and launch.poll() is not None:
            raise RuntimeError('roslaunch exited: %s' % launch.returncode)
        if controller is not None and controller.poll() is not None:
            raise RuntimeError('controller exited: %s' % controller.returncode)
        return current

    def wait_until(predicate, timeout, label):
        end = min(deadline, time.monotonic() + timeout)
        while True:
            current = tick()
            if predicate(current):
                return
            if time.monotonic() >= end:
                raise RuntimeError(label + ' timed out')
            time.sleep(.01)

    def sim_wait(seconds):
        initial = snapshot()['sim']
        wait_until(lambda current: current['sim'] - initial >= seconds,
                   300, phase)

    def run_phase(name, seconds, key=None):
        nonlocal phase
        phase = name
        print('Phase: %s (%.1f simulated seconds)' % (name, seconds), flush=True)
        first, wall_start = snapshot(), time.monotonic()
        first_sample = len(samples)
        try:
            if key is not None:
                os.write(terminal, key)
            tick(force=True)
            sim_wait(seconds)
            tick(force=True)
        finally:
            last, wall_end = snapshot(), time.monotonic()
            sim_elapsed = last['sim'] - first['sim']
            wall_elapsed = wall_end - wall_start
            rows = samples[first_sample:]
            stats = {'sim_seconds': sim_elapsed, 'wall_seconds': wall_elapsed,
                     'rtf': sim_elapsed / wall_elapsed,
                     'start_pose': first['pose'], 'end_pose': last['pose'],
                     'lidar_count': last['lidar_count'] - first['lidar_count'],
                     'imu_count': last['imu_count'] - first['imu_count'],
                     'samples': len(rows)}
            for sensor in ('lidar', 'imu'):
                count = stats[sensor + '_count']
                stats[sensor + '_wall_hz'] = count / wall_elapsed
                stats[sensor + '_sim_hz'] = count / sim_elapsed if sim_elapsed > 0 else None
            if rows:
                stats.update(z_min=min(row['z'] for row in rows),
                             max_abs_roll=max(abs(row['roll']) for row in rows),
                             max_abs_pitch=max(abs(row['pitch']) for row in rows))
            if first['pose'] is not None and last['pose'] is not None:
                stats['dx'] = last['pose']['x'] - first['pose']['x']
                stats['dy'] = last['pose']['y'] - first['pose']['y']
                stats['distance_xy'] = math.hypot(stats['dx'], stats['dy'])
            report['phases'][name] = stats

    try:
        if args.gui or args.rviz:
            graphics = subprocess.run(
                ['python3', '/course/scripts/verification/glx_renderer.py'],
                check=True, capture_output=True, text=True, timeout=20)
            report['graphics'] = json.loads(graphics.stdout)
        launch_args = (['unitree_guide', 'gazeboSim.launch'] if args.launch == 'stock'
                       else ['/course/ros1/mid360/gazebo.launch'])
        command = ['roslaunch'] + launch_args + ['rname:=aliengo', 'paused:=false',
                                                'gui:=' + str(args.gui).lower()]
        if args.launch == 'mid360':
            # This non-graphical trial mounts no display. RGB-D has its own
            # rendering/geometry regression in d435_trial.sh.
            if not args.gui and not args.rviz:
                command.append('d435_enabled:=false')
            command.append('mid360_downsample:=%s' % args.downsample)
            command += ['rviz:=' + str(args.rviz).lower()]
            command += ['spawn_x:=%s' % args.spawn_x, 'spawn_y:=%s' % args.spawn_y,
                        'spawn_z:=%s' % args.spawn_z, 'spawn_yaw:=%s' % args.spawn_yaw]
            if args.world_file:
                command.append('world_file:=' + args.world_file)
        report['command'] = command
        launch = subprocess.Popen(command, stdout=launch_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        wait_until(lambda current: rosgraph.is_master_online(), 90, 'ROS master')
        rospy.init_node('keyboard_regression_trial', disable_signals=True)
        if args.launch == 'mid360':
            tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60))
            tf_listener = tf2_ros.TransformListener(tf_buffer)
        subscribers = [
            rospy.Subscriber('/clock', Clock, on_clock, queue_size=1),
            rospy.Subscriber('/livox/lidar', PointCloud2, on_lidar, queue_size=1),
            rospy.Subscriber('/livox/imu', Imu, on_imu, queue_size=10)]
        rospy.wait_for_service('/gazebo/get_model_state', timeout=90)
        model_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        wait_until(lambda current: current['sim'] is not None and current['sim'] > 0
                   and current['pose'] is not None, 90, 'initial model and clock')
        if args.launch == 'mid360':
            wait_until(lambda current: current['world_tf'] is not None,
                       30, 'world to base TF before controller startup')
        terminal, slave = pty.openpty()
        try:
            controller = subprocess.Popen(['/work/devel/lib/unitree_guide/junior_ctrl'],
                                          stdin=slave, stdout=controller_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            os.close(slave)
        sim_wait(1)
        run_phase('stand', 8, b'2')
        check_pose = True
        tick(force=True)
        run_phase('trot_zero', 2, b'4')
        os.write(terminal, b'w')
        sim_wait(.1)
        run_phase('walk', args.seconds, b'w')
        run_phase('stop', 5, b' ')
        walk, stop = report['phases']['walk'], report['phases']['stop']
        progress = ((walk['end_pose']['x'] - walk['start_pose']['x']) * math.cos(args.spawn_yaw)
                    + (walk['end_pose']['y'] - walk['start_pose']['y']) * math.sin(args.spawn_yaw))
        stop_drift = math.hypot(stop['end_pose']['x'] - stop['start_pose']['x'],
                                stop['end_pose']['y'] - stop['start_pose']['y'])
        report['forward_progress'] = progress
        report['stop_drift'] = stop_drift
        assert progress > args.seconds * .01, 'keyboard did not produce forward motion'
        assert stop_drift < .05, 'robot continued drifting after space'
        if args.launch == 'mid360':
            assert walk['lidar_count'] > 0 and walk['imu_count'] > 0, 'sensor output missing'
            assert any(frame['points'] > 0 for frame in lidar_frames), 'point clouds are empty'
            report['world_tf_motion'] = tf_motion_metrics(
                [row for row in samples if row['phase'] == 'walk'], args.spawn_yaw, progress)
            header = snapshot()['lidar_header']
            assert header is not None and header.stamp.to_nsec() > 0, 'point cloud timestamp missing'
            assert header.frame_id == 'mid360_link', 'unexpected point cloud frame: ' + header.frame_id
            # This must resolve at the actual cloud stamp, never rospy.Time(0).
            transform = tf_buffer.lookup_transform(
                'world', header.frame_id, header.stamp, rospy.Duration(2))
            position = transform.transform.translation
            report['cloud_world_tf'] = {
                'target_frame': 'world', 'source_frame': header.frame_id,
                'cloud_stamp': header.stamp.to_sec(),
                'transform_stamp': transform.header.stamp.to_sec(),
                'world_translation': {'x': position.x, 'y': position.y, 'z': position.z}}
        if args.rviz:
            # Keep the rospy Subscriber handles above alive; graph entries are separate data.
            _, topic_subscribers, services = rosgraph.Master(
                '/keyboard_regression_trial').getSystemState()

            def service_owners(suffix):
                return {node for service, nodes in services
                        if service.endswith(suffix) for node in nodes}

            rviz_nodes = service_owners('/reload_shaders') & service_owners('/load_config')
            lidar_subscribers = {node for topic, nodes in topic_subscribers
                                 if topic == '/livox/lidar' for node in nodes}
            connected = rviz_nodes & lidar_subscribers
            report['rviz_service_owners'] = {
                service: sorted(nodes) for service, nodes in services
                if service.endswith(('/reload_shaders', '/load_config'))}
            report['rviz_nodes'] = sorted(rviz_nodes)
            report['lidar_subscribers'] = sorted(lidar_subscribers)
            report['rviz_lidar_nodes'] = sorted(connected)
            report['rviz_lidar_subscribed'] = bool(connected)
            report['rviz_node_alive'] = {
                node: rosnode.rosnode_ping(node, max_count=1, verbose=False)
                for node in sorted(connected)}
            report['rviz_alive'] = any(report['rviz_node_alive'].values())
            assert report['rviz_alive'], 'No live RViz node subscribes to /livox/lidar'
            from rviz.srv import SendFilePath, SendFilePathRequest
            import yaml

            save_services = [(service, node) for service, nodes in services
                             if service.endswith('/save_config') for node in nodes
                             if report['rviz_node_alive'].get(node)]
            assert save_services, 'Live RViz save_config service missing'
            report['rviz_runtime_views'] = {}
            with tempfile.TemporaryDirectory(prefix='aliengo-trial-rviz-', dir='/tmp') as directory:
                for index, (service, node) in enumerate(sorted(save_services)):
                    path = Path(directory) / ('runtime-%s.rviz' % index)
                    request = SendFilePathRequest()
                    request.path.data = str(path)
                    rospy.wait_for_service(service, timeout=10)
                    response = rospy.ServiceProxy(service, SendFilePath)(request)
                    assert response.success and path.is_file(), 'RViz save_config failed: ' + service
                    saved_config = yaml.safe_load(path.read_text())
                    artifact = args.output / ('rviz-runtime-%s.rviz' % index)
                    shutil.copyfile(path, artifact)
                    metrics = rviz_view_metrics(saved_config)
                    report['rviz_runtime_views'][node] = {
                        **metrics, 'save_service': service, 'config_file': artifact.name}
        if args.check_pause:
            rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
            time.sleep(.2)
            paused = snapshot()['sim']
            time.sleep(.3)
            assert snapshot()['sim'] == paused, 'physics did not pause'
            shutdown_start = time.monotonic()
            os.killpg(controller.pid, signal.SIGINT)
            controller.wait(timeout=3)
            report['paused_shutdown_seconds'] = time.monotonic() - shutdown_start
            report['paused_shutdown_returncode'] = controller.returncode
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = '%s: %s' % (type(error).__name__, error)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        report['phase'] = phase
        for process in (controller, launch):
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=max(.1, min(3, deadline - time.monotonic())))
                except (OSError, subprocess.TimeoutExpired):
                    pass
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except OSError:
                    pass
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
        if terminal is not None:
            os.close(terminal)
        if rospy.core.is_initialized():
            rospy.signal_shutdown('trial finished')
        ogre_log = Path.home() / '.gazebo/ogre.log'
        if ogre_log.is_file():
            shutil.copyfile(str(ogre_log), str(args.output / 'ogre.log'))
        with lock:
            report['lidar_count'] = state['lidar_count']
            report['imu_count'] = state['imu_count']
            report['lidar_frames'] = list(lidar_frames)
            report['walk_lidar_geometry'] = state['walk_lidar_geometry']
        report['wall_seconds'] = time.monotonic() - started
        (args.output / 'samples.json').write_text(json.dumps(samples, indent=2))
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2))
        launch_log.close()
        controller_log.close()
    print('Keyboard trial:', report['result'], report.get('error', ''), flush=True)
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
