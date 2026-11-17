#!/usr/bin/env python3
"""Exercise native fusion and the GBPlanner ROS service with turn clearance.

No custom path/root safety service or acceptance gate is installed. This smoke
uses synthetic measurements and starts no robot, controller or truth adapter.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import xmlrpc.client

os.environ['ROS_MASTER_URI']='http://127.0.0.1:11321'
os.environ['ROS_IP']='127.0.0.1'
import rospy
import tf2_ros
from geometry_msgs.msg import Pose,TransformStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header,String,Float32MultiArray,MultiArrayDimension
from planner_msgs.srv import planner_srv,planner_srvRequest


def wait_until(predicate,description,timeout=20):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.05)
    raise AssertionError('timeout: '+description)


def main():
    libraries=[line for line in subprocess.check_output(
        ['ldd','/gbp/devel/lib/gbplanner/gbplanner_node'],text=True).splitlines()
        if 'libvoxblox' in line]
    assert len(libraries)>=2 and all('/gbp/devel/' in line for line in libraries),libraries
    print('\n'.join(libraries),flush=True)
    processes=[];log=open('/tmp/gbplanner-ros-check.log','w')
    try:
        processes.append(subprocess.Popen(['roscore','-p','11321'],stdout=log,stderr=log,start_new_session=True))
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        def ready():
            try: return master.getUri('/test')[0]==1
            except OSError: return False
        wait_until(ready,'ROS master')
        rospy.init_node('official_backend_check',anonymous=True)
        processes.append(subprocess.Popen(['roslaunch','aliengo_exploration','planner.launch'],stdout=log,stderr=log,start_new_session=True))
        rospy.wait_for_service('/exploration/plan_raw',timeout=30)
        service_names=[entry[0] for entry in master.getSystemState('/test')[2][2]]
        assert '/exploration/check_path' not in service_names,'custom spatial service remains advertised'
        plan=rospy.ServiceProxy('/exploration/plan_raw',planner_srv)
        statuses=[];clouds=[]
        rospy.Subscriber('/exploration/map_status',String,lambda msg:statuses.append(json.loads(msg.data)))
        rospy.Subscriber('/gbplanner/tsdf_pointcloud',PointCloud2,clouds.append,queue_size=2)
        cloud_pub=rospy.Publisher('/exploration/cloud',PointCloud2,queue_size=10)
        terrain_pub=rospy.Publisher('/exploration/elevation_map',GridMap,queue_size=1,latch=True)
        odom_pub=rospy.Publisher('/exploration/base_odom',Odometry,queue_size=1,latch=True)
        wait_until(lambda:cloud_pub.get_num_connections()>0 and odom_pub.get_num_connections()>0,'planner input subscribers')
        root=Pose();root.position.z=.43;root.orientation.w=1
        odom=Odometry();odom.header.frame_id='lio_map';odom.child_frame_id='base';odom.pose.pose=root
        # Official startup resets/augments the map on first odometry. Send it
        # before the observations whose distinct origins this fixture examines.
        odom_pub.publish(odom);time.sleep(.3)
        broadcaster=tf2_ros.StaticTransformBroadcaster()
        lidar=TransformStamped();lidar.header.frame_id='lio_map';lidar.child_frame_id='mid360_link'
        lidar.transform.translation.x=-5;lidar.transform.translation.z=1;lidar.transform.rotation.w=1
        broadcaster.sendTransform([lidar]);time.sleep(.3)
        def send(frame,stamp,points):
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.from_sec(stamp),frame_id=frame),points))
        def count(frame):
            return statuses[-1]['sources'].get(frame,{}).get('count',0) if statuses else 0
        camera='d435_depth_optical_frame';lidar_frame='mid360_link'
        send(camera,20,[(2,0,0)]);send(lidar_frame,20,[(2,0,0)])
        wait_until(lambda:count(lidar_frame)==1,'lidar despite blocked camera TF')
        assert count(camera)==0
        camera_tf=TransformStamped();camera_tf.header.frame_id='lio_map';camera_tf.child_frame_id=camera
        camera_tf.transform.translation.x=5;camera_tf.transform.translation.z=1
        camera_tf.transform.rotation.z=math.sin(math.pi/4);camera_tf.transform.rotation.w=math.cos(math.pi/4)
        broadcaster.sendTransform([lidar,camera_tf])
        wait_until(lambda:count(camera)==1,'queued camera TF retry without a new cloud')
        send(camera,10,[(2,0,0)]);send(lidar_frame,10,[(2,0,0)])
        wait_until(lambda:count(camera)==2 and count(lidar_frame)==2,'equal and late source stamps')
        def voxels():
            return list(point_cloud2.read_points(clouds[-1],field_names=['x','y','z','intensity'],skip_nans=True)) if clouds else []
        wait_until(lambda:any(4.9<p[0]<5.1 and .9<p[1]<1.1 for p in voxels()),'rotated native camera ray')
        points=voxels()
        assert any(-4.1<p[0]<-3.9 and abs(p[1])<.15 for p in points)
        # Official root augmentation occupies only the small box near x=0;
        # incorrectly tracing camera rays from map origin would cross x=1..4.
        assert not any(1<p[0]<4 for p in points),'sensor rays incorrectly originate in map frame'
        terrain=GridMap();terrain.info.header.frame_id='lio_map';terrain.info.header.stamp=rospy.Time.now()
        terrain.info.resolution=.1;terrain.info.length_x=8;terrain.info.length_y=8
        terrain.info.pose.orientation.w=1;terrain.layers=['elevation'];terrain.basic_layers=['elevation']
        data=Float32MultiArray();data.layout.dim=[MultiArrayDimension('column_index',80,6400),MultiArrayDimension('row_index',80,80)]
        data.data=[0.]*6400;terrain.data=[data];terrain_pub.publish(terrain)
        plane=[(8,y*.025,z*.025) for y in range(-48,49) for z in range(-88,25)]
        send(lidar_frame,30,plane)
        wait_until(lambda:count(lidar_frame)==3,'actual TSDF free-volume observations',30)
        time.sleep(.3)
        request=planner_srvRequest();request.header.frame_id='lio_map';request.bound_mode=3
        paths=[];rejected=0
        # A sampled graph path can now be rejected by its actual final turn.
        # Match the coordinator's bounded retry policy while still requiring
        # two usable paths; an empty response must never be followed as a path.
        for attempt in range(6):
            response=plan(request)
            if not response.path:
                rejected+=1
                continue
            assert len(response.path)>1,'planner returned a one-pose path'
            assert all(math.isfinite(v) for p in response.path for v in [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            length=sum(math.sqrt((a.position.x-b.position.x)**2+(a.position.y-b.position.y)**2+(a.position.z-b.position.z)**2) for a,b in zip(response.path,response.path[1:]))
            paths.append({'poses':len(response.path),'length':length})
            if len(paths)==2: break
        assert len(paths)==2,'planner did not return two usable paths within bounded retries'
        print(json.dumps({'passed':True,'spatial_logic':'pinned GBPlanner with oriented body/turn collision geometry fixes',
            'custom_check_path_service':False,'sources':statuses[-1]['sources'],'paths':paths,'rejected_candidates':rejected,
            'checks':['native translated and rotated ray origins','equal/late source stamps','independent TF wait/retry','official repeated planner_srv paths']}))
    finally:
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in reversed(processes):
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL)
        log.close()
        if sys.exc_info()[0]: print(open('/tmp/gbplanner-ros-check.log').read()[-18000:],file=sys.stderr)

if __name__=='__main__': main()
