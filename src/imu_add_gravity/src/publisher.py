#!/usr/bin/env python3

from sensor_msgs.msg import Imu
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np


def publishModifiedImu(msg: Imu):
    global odom_pub
    rot = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    gravity = np.array([0.0, 0.0, 9.81])
    gravity = rot.apply(gravity)
    msg.linear_acceleration.x += gravity[0]
    msg.linear_acceleration.y += gravity[1]
    msg.linear_acceleration.z += gravity[2]
    odom_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('imu_publisher')
    odom_pub = rospy.Publisher(name='/out', data_class=Imu, queue_size=1)
    odom_sub = rospy.Subscriber(name='/in', data_class=Imu, callback=publishModifiedImu)
    rospy.spin()
