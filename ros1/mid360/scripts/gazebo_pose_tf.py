#!/usr/bin/env python3
"""发布 Gazebo 仿真真值 world→base；这不是运动控制器或 SLAM 估计。"""
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix


def state_messages(state):
    """同一次查询的位姿、速度和仿真时间用于 TF 与真值里程计。"""
    transform = TransformStamped()
    transform.header = Header(stamp=state.header.stamp, frame_id='world')
    transform.child_frame_id = 'base'
    p, q = state.pose.position, state.pose.orientation
    transform.transform.translation = Vector3(p.x, p.y, p.z)
    transform.transform.rotation = q

    odom = Odometry()
    odom.header = transform.header
    odom.child_frame_id = 'base'
    odom.pose.pose = state.pose
    # 服务返回 world 系速度；Odometry 的 twist 应表达在 child_frame_id=base 中。
    world_to_base = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].T
    linear, angular = state.twist.linear, state.twist.angular
    odom.twist.twist.linear = Vector3(*world_to_base.dot([linear.x, linear.y, linear.z]))
    odom.twist.twist.angular = Vector3(*world_to_base.dot([angular.x, angular.y, angular.z]))
    return transform, odom


def main():
    rospy.init_node('gazebo_ground_truth')
    model_name = rospy.get_param('~model_name', 'aliengo_gazebo')
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState, persistent=True)
    broadcaster = tf2_ros.TransformBroadcaster()
    publisher = rospy.Publisher('/sim/ground_truth/odom', Odometry, queue_size=1)
    rate = rospy.Rate(30, reset=True)
    previous_stamp = None
    while not rospy.is_shutdown():
        state = get_state(model_name, 'world')
        # 启动时模型可能尚未生成；暂停期间不要反复发布同一时间戳。
        if state.success and state.header.stamp.to_nsec() > 0 and state.header.stamp != previous_stamp:
            transform, odom = state_messages(state)
            broadcaster.sendTransform(transform)
            publisher.publish(odom)
            previous_stamp = state.header.stamp
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
