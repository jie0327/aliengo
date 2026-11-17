#!/usr/bin/env python3
"""A missing camera must cause a bounded explicit failure, not a frozen estimator."""
import os
import subprocess
import tempfile
import time
import xmlrpc.client


def wait_until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def main():
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11311'
    os.environ['ROS_HOSTNAME'] = 'localhost'
    master = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.STDOUT)
    mapper = None
    with tempfile.TemporaryFile(mode='w+') as output:
        try:
            def master_ready():
                try:
                    return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/overflow_test')[0] == 1
                except OSError:
                    return False
            assert wait_until(master_ready, 10), 'ROS master did not start'
            import rospy
            from sensor_msgs.msg import Imu, PointCloud2
            from sensor_msgs.point_cloud2 import create_cloud_xyz32
            from std_msgs.msg import Header
            rospy.init_node('fast_livo_overflow_test', disable_signals=True)
            identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            parameters = {
                'common/img_en': 1, 'common/lidar_en': 1,
                'common/lid_topic': '/overflow/points',
                'common/imu_topic': '/overflow/imu',
                'common/img_topic': '/overflow/absent_image',
                'imu/imu_en': True, 'preprocess/lidar_type': 8,
                'preprocess/point_filter_num': 1,
                'extrin_calib/extrinsic_T': [0, 0, 0],
                'extrin_calib/extrinsic_R': identity,
                'extrin_calib/Pcl': [0, 0, 0], 'extrin_calib/Rcl': identity,
                'laserMapping/cam_model': 'Pinhole',
                'laserMapping/cam_width': 640, 'laserMapping/cam_height': 360,
                'laserMapping/cam_fx': 400.0, 'laserMapping/cam_fy': 400.0,
                'laserMapping/cam_cx': 320.0, 'laserMapping/cam_cy': 180.0,
            }
            for name, value in parameters.items():
                rospy.set_param('/' + name, value)
            mapper = subprocess.Popen(['/fast_livo/devel/lib/fast_livo/fastlivo_mapping'],
                                      stdout=output, stderr=subprocess.STDOUT)
            publisher = rospy.Publisher('/overflow/points', PointCloud2, queue_size=10)
            imu_publisher = rospy.Publisher('/overflow/imu', Imu, queue_size=10)
            assert wait_until(lambda: publisher.get_num_connections() > 0, 10), 'mapper did not subscribe'
            for _ in range(160):
                if mapper.poll() is not None:
                    break
                message = create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id='mid360_link'),
                                            [(2, 0, 0), (2, 1, 0)])
                publisher.publish(message)
                sample = Imu(header=message.header)
                sample.linear_acceleration.z = 9.81
                imu_publisher.publish(sample)
                time.sleep(0.02)
            assert wait_until(lambda: mapper.poll() is not None, 3), 'full LiDAR queue left estimator permanently waiting'
            assert mapper.returncode == 0, 'mapper did not exit cleanly for map preservation'
            output.seek(0)
            assert 'LiDAR buffer overflow' in output.read(), 'shutdown did not identify sensor backlog'
            print('FAST-LIVO2 absent-camera overflow regression passed')
        finally:
            for process in (mapper, master):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == '__main__':
    main()
