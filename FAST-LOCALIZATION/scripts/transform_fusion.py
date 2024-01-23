#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread as thread
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_map_to_odom = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'camera_init', 'map')
        if cur_odom is not None:
            # odometry
            localization = Odometry()
            # 주어진 roll, pitch, yaw 각도
            roll = 0  # 예시 값
            pitch = 0  # 예시 값
            #yaw = -4.1000291187947155  # 예시 값
            yaw = 0
            
            # roll, pitch, yaw 각도를 쿼터니언으로 변환
            quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            # 쿼터니언을 회전 행렬로 변환
            R = tf.transformations.quaternion_matrix(quat)[:3, :3]
            T_map1_to_map = np.eye(4)
            # T_map1_to_map[0, 3] = -417.6195511179997  # x-offset
            # T_map1_to_map[1, 3] = 903.6885174906914  # y-offset
            # T_map1_to_map[:3, :3] = R
            T_map1_to_map[0, 3] = 0 # x-offset
            T_map1_to_map[1, 3] = 0 # y-offset
            T_map1_to_map[:3, :3] = R
            T_odom_to_base_link = pose_to_mat(cur_odom)

            T_map1_to_odom = np.matmul(T_map1_to_map, T_map_to_odom)
            T_map1_to_base_link = np.matmul(T_map1_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map1_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map1_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map1'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)



def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    thread.start_new_thread(transform_fusion, ())

    rospy.spin()
