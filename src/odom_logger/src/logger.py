#!/usr/bin/env python3

import csv
from geometry_msgs.msg import PoseStamped, Point, Vector3
from nav_msgs.msg import Odometry
import rospy
import tf2_geometry_msgs
import tf2_ros
from typing import Any, Optional


def point2vector(p: Point) -> Vector3:
    return Vector3(x=p.x, y=p.y, z=p.z)


def logOdom(msg: Odometry, args: Any):
    global last_msg
    # Cannot calculate the velocity for the first frame, just store it
    if last_msg is None:
        last_msg = msg
        return

    transform = args['tf_buffer'].lookup_transform(target_frame='base_link',
                                                   source_frame='world',
                                                   time=rospy.Time(0))
    last_pose_global = PoseStamped()
    last_pose_global.header = msg.header
    last_pose_global.pose = last_msg.pose.pose
    current_pose_global = PoseStamped()
    current_pose_global.header = msg.header
    current_pose_global.pose = msg.pose.pose
    # It does not make sense here!
    last_pose_odom = tf2_geometry_msgs.do_transform_pose(pose=last_pose_global, transform=transform)
    current_pose_odom = tf2_geometry_msgs.do_transform_pose(pose=current_pose_global, transform=transform)
    dx_odom = current_pose_odom.pose.position.x - last_pose_odom.pose.position.x
    dy_odom = current_pose_odom.pose.position.y - last_pose_odom.pose.position.y
    dz_odom = current_pose_odom.pose.position.z - last_pose_odom.pose.position.z

    dt = msg.header.stamp.to_sec() - last_msg.header.stamp.to_sec()

    vx_odom = dx_odom / dt
    vy_odom = dy_odom / dt
    vz_odom = dz_odom / dt
    # Transformed from world to odom so that the local velocity can be estimated

    # with open(args['log_file'], mode='a') as log_file:
    #   sensor_write = csv.writer(log_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #   write_to_log = sensor_write.writerow(
    #   [date_now(), time_now(), get_temp(), get_pressure(), get_lux(), cpu_temperature()])
    #   return(write_to_log)'''
    #rospy.loginfo(f'{vx_odom=}, {vy_odom=}, {vz_odom=}')
    rospy.loginfo(f'\n{dx_odom=}\n{dy_odom=}\n{dz_odom=}\n{dt=}\n'
                  f'{vx_odom=}\n{vy_odom=}\n{vz_odom=}')
    last_msg = msg


if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    tf_buffer = tf2_ros.Buffer()
    last_msg: Optional[Odometry] = None
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    callback_args = {'log_file': '/home/mikolaj/Projects/catkin_vins',
                     'tf_buffer': tf_buffer}
    odom_sub = rospy.Subscriber(name='/odom',
                                data_class=Odometry,
                                callback=logOdom,
                                callback_args=callback_args)
    rospy.spin()
