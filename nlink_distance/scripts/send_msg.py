#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
import tf.transformations
import math
import numpy as np

def create_odometry_message():
    # 创建Odometry消息
    odom_msg = Odometry()

    # 设置消息头
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # 设置位置 (例如: x, y, z 均为0)
    position = Point(1.0, 2.0, 3.0)

    # 设置方向 (例如: 四元数表示)
    orientation = Quaternion(*tf.transformations.quaternion_from_euler(math.pi / 2, math.pi / 2, math.pi / 2))

    # 创建姿态消息
    pose = Pose(position, orientation)

    # 设置速度 (例如: 线速度和角速度均为0)
    linear = Vector3(0.0, 0.0, 0.0)
    angular = Vector3(0.0, 0.0, 0.0)
    twist = Twist(linear, angular)

    # 设置姿态和速度的协方差矩阵
    # 示例协方差矩阵 (对角线上的值可以根据实际情况调整)
    # 1 表示主机，0 表示从机
    odom_msg.pose.covariance = [
        1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    ]
    odom_msg.pose.covariance = np.array(odom_msg.pose.covariance)

    odom_msg.twist.covariance = [
        1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0
    ]
    odom_msg.twist.covariance = np.array(odom_msg.twist.covariance)

    # 将姿态和速度设置到Odometry消息中
    odom_msg.pose.pose = pose
    odom_msg.twist.twist = twist

    return odom_msg

def odometry_publisher():
    flag = True
    # 初始化ROS节点
    rospy.init_node('odometry_publisher', anonymous=True)

    # 创建一个发布者，发布到/communicate_server话题，消息类型为Odometry
    pub = rospy.Publisher('/communicate_server', Odometry, queue_size=10)

    # 设置发布频率 (例如: 10Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # 创建Odometry消息
        odom_msg = create_odometry_message()
        # 发布消息
        pub.publish(odom_msg)
        # 按照设定频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass
