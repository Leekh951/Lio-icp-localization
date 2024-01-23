#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import
import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

if __name__ == '__main__':
   
    # parser.add_argument('x', type=float)
    # parser.add_argument('y', type=float)
    # parser.add_argument('z', type=float)
    # parser.add_argument('yaw', type=float)
    # parser.add_argument('pitch', type=float)
    # parser.add_argument('roll', type=float)

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    rospy.init_node('publish_initial_pose')
    pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    xyz = [x, y, z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    initial_pose.header.stamp = rospy.Time().now()
    initial_pose.header.frame_id = 'map'
    rospy.sleep(1)
    rospy.loginfo('Initial Pose: {} {} {} {} {} {}'.format(
        x, y, z, yaw, pitch, roll, ))
    pub_pose.publish(initial_pose)
