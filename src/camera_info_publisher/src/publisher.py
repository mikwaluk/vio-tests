#!/usr/bin/env python3

import yaml

import rospy
from sensor_msgs.msg import CameraInfo


def getCameraParams(config_file: str) -> CameraInfo:
    # Load data from file
    with open(config_file, 'r') as file_handle:
        calib_data = yaml.load(stream=file_handle, Loader=yaml.FullLoader)
    # Parse
    camera_info_msg = CameraInfo()

    camera_info_msg.width = calib_data['image_width']
    camera_info_msg.height = calib_data['image_height']
    camera_info_msg.K = calib_data['camera_matrix']['data']
    camera_info_msg.D = calib_data['distortion_coefficients']['data']
    camera_info_msg.R = calib_data['rectification_matrix']['data']
    camera_info_msg.P = calib_data['projection_matrix']['data']
    camera_info_msg.distortion_model = calib_data['distortion_model']
    return camera_info_msg


if __name__ == '__main__':
    rospy.init_node('camera_info_publisher')
    publisher = rospy.Publisher(name='/camera_info', data_class=CameraInfo, latch=True, queue_size=1)
    config_file = rospy.get_param(param_name='~camera_info_file')
    info_message = getCameraParams(config_file=config_file)
    camera_frame = rospy.get_param(param_name='~camera_frame')
    info_message.header.frame_id = camera_frame
    publisher.publish(info_message)
    rospy.spin()
