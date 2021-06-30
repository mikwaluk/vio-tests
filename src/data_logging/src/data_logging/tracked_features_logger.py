#!/usr/bin/env python3

import csv
import rospy
from logging_msgs.msg import TrackedFeatures


class Evaluator:
    def __init__(self, features_topic: str, log_filename: str) -> None:
        self._tracked_features_sub = rospy.Subscriber(name=features_topic, data_class=TrackedFeatures,
                                                      callback=self.handleNewFrame, queue_size=1)
        header = ['timestamp', 'tracked_features']
        self._log_file = open(file=log_filename, mode='w')
        self._csv_writer = csv.writer(self._log_file, delimiter=',')
        self._csv_writer.writerow(header)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self) -> None:
        if not self._log_file.closed:
            self._log_file.close()

    def handleNewFrame(self, msg: TrackedFeatures) -> None:
        rospy.loginfo('Received new message')
        self._csv_writer.writerow([msg.header.stamp.to_sec(), msg.tracked_features.data])


if __name__ == '__main__':
    rospy.init_node('feature_tracker_evaluation')
    log_filename = rospy.get_param('~log_filename')
    evaluator = Evaluator(features_topic='/tracked_features', log_filename=log_filename)

    rospy.spin()
