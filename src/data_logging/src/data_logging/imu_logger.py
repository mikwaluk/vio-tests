#!/usr/bin/env python3

import csv

import rospy

from sensor_msgs.msg import Imu
from data_logging.odom_logger import euler_from_quaternion


class ImuLogger:
    def __init__(self, filename: str) -> None:
        self._log_file = open(file=filename, mode='w')
        header = ['timestamp',
                  'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                  'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                  'roll', 'pitch', 'yaw']
        self._csv_writer = csv.writer(self._log_file, delimiter=',')
        self._csv_writer.writerow(header)
        rospy.loginfo(f'Started logging to file {filename}')
        # rospy.on_shutdown(h=self.shutdown)
        self._bias_sub = rospy.Subscriber(name='/imu', data_class=Imu, callback=self.logImu)

    # def shutdown(self):
    #    if not self._log_file.closed:
    #        self._log_file.close()

    def logImu(self, msg: Imu) -> None:
        lin_acc_x = msg.linear_acceleration.x
        lin_acc_y = msg.linear_acceleration.y
        lin_acc_z = msg.linear_acceleration.z
        ang_vel_x = msg.angular_velocity.x
        ang_vel_y = msg.angular_velocity.y
        ang_vel_z = msg.angular_velocity.z
        r, p, y = euler_from_quaternion(x=msg.orientation.x, y=msg.orientation.y,
                                        z=msg.orientation.z, w=msg.orientation.w)
        self._csv_writer.writerow([msg.header.stamp.to_sec(),
                                   lin_acc_x, lin_acc_y, lin_acc_z,
                                   ang_vel_x, ang_vel_y, ang_vel_z,
                                   r, p, y])
        # rospy.loginfo(f'\n{acc_x=}\n{acc_y=}\n{acc_z=}\n'
        #              f'{gyro_x=}\n{gyro_y=}\n{gyro_z=}')


if __name__ == '__main__':
    rospy.init_node('imu_logger')
    log_filename = rospy.get_param(param_name='~log_filename')
    logger = ImuLogger(filename=log_filename)
    rospy.spin()
