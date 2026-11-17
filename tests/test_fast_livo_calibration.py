"""Exercise calibration passed to FAST-LIVO2, independently of ROS transport."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ros1/fast_livo/scripts/calibration_core.py'


class CalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SOURCE.exists():
            raise AssertionError('runtime FAST-LIVO2 calibration is not implemented')
        spec = importlib.util.spec_from_file_location('fast_livo_calibration', SOURCE)
        cls.core = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.core)

    def samples(self):
        info = dict(width=640, height=360,
                    K=[465.601475318367, 0, 320, 0, 465.601475318367, 180, 0, 0, 1],
                    D=[0, 0, 0, 0, 0], R=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                    P=[465.601475318367, 0, 320, 0, 0, 465.601475318367, 180, 0, 0, 0, 1, 0],
                    distortion_model='plumb_bob', frame_id='d435_color_optical_frame', stamp=12.,
                    binning_x=0, binning_y=0, roi=dict(x_offset=0, y_offset=0, width=0, height=0))
        image = dict(width=640, height=360, encoding='rgb8',
                     frame_id='d435_color_optical_frame', stamp=12.)
        return info, image

    def test_uses_actual_camera_calibration_without_second_resize(self):
        # A stale 1280x720 calibration or the upstream default scale=.5 must fail.
        info, image = self.samples()
        parameters = self.core.camera_parameters(info, image, now=12.1)
        self.assertEqual(parameters['cam_width'], 640)
        self.assertEqual(parameters['cam_height'], 360)
        self.assertEqual(parameters['scale'], 1.)
        self.assertAlmostEqual(parameters['cam_fx'], 465.601475318367)
        self.assertEqual((parameters['cam_cx'], parameters['cam_cy']), (320., 180.))

    def test_camera_input_rejects_invalid_or_incompatible_calibration(self):
        info, image = self.samples()
        bad = [dict(K=[0]*9), dict(K=[float('nan')]+info['K'][1:]),
               dict(width=1280), dict(distortion_model='equidistant'),
               dict(D=[0, 0, 0, 0, .1]), dict(stamp=0), dict(stamp=10),
               dict(frame_id='d435_depth_optical_frame'), dict(binning_x=2),
               dict(roi=dict(x_offset=10, y_offset=0, width=320, height=180)),
               dict(R=[0, -1, 0, 1, 0, 0, 0, 0, 1]),
               dict(P=[465.6, 0, 320, .05, 0, 465.6, 180, 0, 0, 0, 1, 0])]
        for change in bad:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.core.camera_parameters(dict(info, **change), image, now=12.1)
        for change in (dict(height=720), dict(stamp=12.01), dict(encoding='16UC1')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.core.camera_parameters(info, dict(image, **change), now=12.1)
        with self.assertRaises(ValueError):
            self.core.camera_parameters(info, image, now=13.)
        with self.assertRaises(ValueError):
            self.core.camera_parameters(info, image, now=11.)

    def test_too_small_image_cannot_reach_upstream_pyramid_and_grid(self):
        info, image = self.samples()
        info.update(width=16, height=16, K=[20, 0, 8, 0, 20, 8, 0, 0, 1],
                    P=[20, 0, 8, 0, 0, 20, 8, 0, 0, 0, 1, 0])
        image.update(width=16, height=16)
        with self.assertRaises(ValueError):
            self.core.camera_parameters(info, image, now=12.1)

    def test_optical_projection_and_imu_offset_keep_correct_direction(self):
        # Translation sign reversal or transposed camera rotation changes this pixel.
        imu_from_lidar = np.eye(4)
        imu_from_lidar[:3, 3] = [-.011, -.02329, .04412]
        camera_from_lidar = np.array([[0, -1, 0, .015], [0, 0, -1, -.133],
                                      [1, 0, 0, -.07], [0, 0, 0, 1.]])
        values = self.core.extrinsic_parameters(imu_from_lidar, camera_from_lidar)
        camera = np.asarray(values['Rcl']).reshape(3, 3) @ [2., 0, 0] + values['Pcl']
        np.testing.assert_allclose(camera, [.015, -.133, 1.93], atol=1.e-12)
        imu = np.asarray(values['extrinsic_R']).reshape(3, 3) @ [2., 0, 0] + values['extrinsic_T']
        np.testing.assert_allclose(imu, [1.989, -.02329, .04412], atol=1.e-12)
        # FAST-LIVO2 composes R_CI=R_CL*R_IL.T, t_CI=t_CL-R_CI*t_IL.
        r_ci = np.asarray(values['Rcl']).reshape(3, 3) @ np.asarray(values['extrinsic_R']).reshape(3, 3).T
        t_ci = np.asarray(values['Pcl']) - r_ci @ values['extrinsic_T']
        np.testing.assert_allclose(t_ci, [-.00829, -.08888, -.059], atol=1.e-12)

    def test_invalid_tf_cannot_become_extrinsics(self):
        for value in (np.zeros((4, 4)), np.diag([-1., 1, 1, 1]),
                      np.diag([2., 1, 1, 1]), np.full((4, 4), float('nan'))):
            with self.subTest(value=value.tolist()), self.assertRaises(ValueError):
                self.core.extrinsic_parameters(value, np.eye(4))

    def test_ros_bootstrap_sets_live_parameters_in_estimator_namespace(self):
        # Real ROS transport catches namespace/remapping and exec handoff mistakes.
        if not shutil.which('docker'):
            self.skipTest('Docker unavailable')
        available = subprocess.run(['docker', 'image', 'inspect', 'aliengo-learning:noetic'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if available.returncode:
            self.skipTest('Noetic image unavailable')
        program = r'''
import json, os, pathlib, subprocess, sys, time
import numpy as np
import rosgraph, rospy, tf2_ros
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

master = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
child = None
try:
    deadline = time.monotonic() + 15
    while not rosgraph.is_master_online():
        assert time.monotonic() < deadline, 'ROS master startup timed out'
        time.sleep(.05)
    rospy.init_node('camera_fixture')
    rospy.set_param('/use_sim_time', True)
    rospy.set_param('/fast_livo/common/img_topic', '/camera/color/image_raw')
    rospy.set_param('/fast_livo/calibration/timeout_wall', 10.)
    rospy.set_param('/fast_livo/pcd_save/output_dir', '/tmp/generated-maps')
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
    info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
    transforms = []
    for parent, child_frame, xyz, quat in (
        ('base', 'mid360_link', [.25, 0, .103], [0, 0, 0, 1]),
        ('mid360_link', 'mid360_imu_link', [.011, .02329, -.04412], [0, 0, 0, 1]),
        ('base', 'd435_color_optical_frame', [.32, .015, -.03], [-.5, .5, -.5, .5])):
        value = TransformStamped()
        value.header.frame_id, value.child_frame_id = parent, child_frame
        p, q = value.transform.translation, value.transform.rotation
        p.x, p.y, p.z = xyz
        q.x, q.y, q.z, q.w = quat
        transforms.append(value)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    probe = pathlib.Path('/tmp/estimator_probe.py')
    probe.write_text("""import json, pathlib, rospy
rospy.init_node('laserMapping')
assert rospy.get_name() == '/fast_livo/laserMapping', rospy.get_name()
data = {'camera': rospy.get_param('laserMapping'),
        'extrinsics': rospy.get_param('extrin_calib'),
        'loaded': rospy.get_param('calibration/loaded')}
pathlib.Path('/tmp/calibration_probe.json').write_text(json.dumps(data))
""")
    child = subprocess.Popen(['/course/ros1/fast_livo/scripts/calibrated_exec.py',
                              sys.executable, str(probe), '__ns:=/fast_livo', '__name:=laserMapping'])
    info = CameraInfo()
    info.width, info.height = 640, 360
    info.header.frame_id = 'd435_color_optical_frame'
    info.distortion_model = 'plumb_bob'
    info.D = [0.] * 5
    info.K = [465.601475318367, 0, 320, 0, 465.601475318367, 180, 0, 0, 1]
    info.R = [1., 0, 0, 0, 1, 0, 0, 0, 1]
    info.P = [465.601475318367, 0, 320, 0, 0, 465.601475318367, 180, 0, 0, 0, 1, 0]
    image = Image()
    image.width, image.height, image.encoding, image.step = 640, 360, 'rgb8', 1920
    image.data = bytes(image.step * image.height)
    count = 0
    while child.poll() is None:
        assert time.monotonic() < deadline, 'calibration bootstrap did not exec estimator'
        stamp = rospy.Time.from_sec(12. + count * .02)
        # Sensor and clock callbacks are independent. A slightly future image
        # must wait for /clock rather than fail the calibration bootstrap.
        clock_pub.publish(Clock(clock=rospy.Time.from_sec(stamp.to_sec() - (.2 if count < 40 else 0))))
        info.header.stamp = stamp
        image.header = info.header
        info_pub.publish(info)
        image_pub.publish(image)
        count += 1
        time.sleep(.03)
    assert child.returncode == 0, 'bootstrap exit ' + str(child.returncode)
    data = json.loads(pathlib.Path('/tmp/calibration_probe.json').read_text())
    assert data['camera']['cam_width'] == 640 and data['camera']['scale'] == 1.
    assert abs(data['camera']['cam_fx'] - 465.601475318367) < 1.e-9
    np.testing.assert_allclose(data['extrinsics']['extrinsic_T'], [-.011, -.02329, .04412], atol=1.e-10)
    np.testing.assert_allclose(data['extrinsics']['Rcl'], [0, -1, 0, 0, 0, -1, 1, 0, 0], atol=1.e-10)
    np.testing.assert_allclose(data['extrinsics']['Pcl'], [.015, -.133, -.07], atol=1.e-10)
    assert data['loaded']['image_frame'] == 'd435_color_optical_frame'
    assert pathlib.Path('/tmp/generated-maps').is_dir()
finally:
    if child is not None and child.poll() is None:
        child.kill()
        child.wait()
    master.terminate()
    master.wait(timeout=5)
'''
        result = subprocess.run([
            'docker', 'run', '--rm', '-i', '--init', '--network', 'none', '--memory=512m', '--cpus=1',
            '--mount', f'type=bind,source={ROOT},target=/course,readonly',
            '-e', 'HOME=/tmp/calibration-test', '-e', 'ROS_HOSTNAME=localhost',
            '-e', 'ROS_IP=127.0.0.1', '--entrypoint', 'bash', 'aliengo-learning:noetic', '-c',
            'source /opt/ros/noetic/setup.bash\nexec python3 -'],
            input=program, capture_output=True, text=True, timeout=35)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
