#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rospy
import tf2_ros


def publishOdom(msg: Odometry):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = 'odom'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    odom_sub = rospy.Subscriber(name='/odom', data_class=Odometry, callback=publishOdom)
    rospy.spin()
