#!/usr/bin/env python3

import csv

import rospy

from logging_msgs.msg import Bias


class BiasLogger:
    def __init__(self, filename: str) -> None:
        self._log_file = open(file=filename, mode='w')
        header = ['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
        self._csv_writer = csv.writer(self._log_file, delimiter=',')
        self._csv_writer.writerow(header)
        rospy.loginfo(f'Started logging to file {filename}')
        # rospy.on_shutdown(h=self.shutdown)
        self._bias_sub = rospy.Subscriber(name='/bias', data_class=Bias, callback=self.logBias)

    # def shutdown(self):
    #    if not self._log_file.closed:
    #        self._log_file.close()

    def logBias(self, msg: Bias) -> None:
        acc_x = msg.bias_acc.x
        acc_y = msg.bias_acc.y
        acc_z = msg.bias_acc.z
        gyro_x = msg.bias_gyro.x
        gyro_y = msg.bias_gyro.y
        gyro_z = msg.bias_gyro.z
        self._csv_writer.writerow([msg.header.stamp.to_sec(), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
        # rospy.loginfo(f'\n{acc_x=}\n{acc_y=}\n{acc_z=}\n'
        #              f'{gyro_x=}\n{gyro_y=}\n{gyro_z=}')


if __name__ == '__main__':
    rospy.init_node('bias_logger')
    log_filename = rospy.get_param(param_name='~log_filename')
    logger = BiasLogger(filename=log_filename)
    rospy.spin()
