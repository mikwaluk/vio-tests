#!/usr/bin/env python3

import csv
from geometry_msgs.msg import PoseStamped, Point, Vector3, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import tf2_geometry_msgs
import tf2_ros
import math
from typing import Any, Optional, Tuple


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)


class OdomLogger:
    def __init__(self, filename: str, transform_v_to_odom: bool) -> None:
        self._log_file = open(file=filename, mode='w')
        header = ['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']
        self._csv_writer = csv.writer(self._log_file, delimiter=',')
        self._csv_writer.writerow(header)
        rospy.loginfo(f'Started logging to file {filename}')
        self._transform_v_to_odom = transform_v_to_odom
        # rospy.on_shutdown(h=self.shutdown)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        #self._last_msg: Optional[Odometry] = None
        self._odom_sub = rospy.Subscriber(name='/odom', data_class=Odometry, callback=self.logOdom)

    # def shutdown(self):
    #    if not self._log_file.closed:
    #        self._log_file.close()

    def logOdom(self, msg: Odometry) -> None:

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        if self._transform_v_to_odom:
            # VINS estimates the velocity in global frame, therefore transform it from world to odom
            world_to_odom: TransformStamped = self._tf_buffer.lookup_transform(target_frame='base_link',
                                                                               source_frame='world',
                                                                               time=rospy.Time(0))
            # rospy.loginfo(f'before: {vx=}, {vy=}, {vz=}')
            velocity_vector = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            rotation = Rotation.from_quat([world_to_odom.transform.rotation.x, world_to_odom.transform.rotation.y,
                                          world_to_odom.transform.rotation.z, world_to_odom.transform.rotation.w])
            # rospy.loginfo(rotation.as_matrix())
            transformed_v = np.dot(rotation.as_matrix(), velocity_vector)
            vx = transformed_v[0]
            vy = transformed_v[1]
            vz = transformed_v[2]
            # rospy.loginfo(f'after: {vx=}, {vy=}, {vz=}')

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        r, p, y = euler_from_quaternion(x=msg.pose.pose.orientation.x, y=msg.pose.pose.orientation.y,
                                        z=msg.pose.pose.orientation.z, w=msg.pose.pose.orientation.w)
        self._csv_writer.writerow([msg.header.stamp.to_sec(), x, y, z, vx, vy, vz, r, p, y])
        # rospy.loginfo(f'\n{dx_odom=}\n{dy_odom=}\n{dz_odom=}\n{dt=}\n'
        #              f'{vx_odom=}\n{vy_odom=}\n{vz_odom=}')


if __name__ == '__main__':
    rospy.init_node('odom_logger')
    transform_v_to_odom = rospy.get_param(param_name='~transform_vel_to_odom', default=False)
    log_filename = rospy.get_param(param_name='~log_filename')
    logger = OdomLogger(filename=log_filename, transform_v_to_odom=transform_v_to_odom)
    rospy.spin()
