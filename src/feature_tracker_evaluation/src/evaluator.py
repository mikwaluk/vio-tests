#!/usr/bin/env python3

import csv
from nav_msgs.msg import Odometry
from queue import Queue
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class Evaluator:
    def __init__(self, cloud_topic: str, log_filename: str) -> None:
        self._feature_cloud_sub = rospy.Subscriber(name=cloud_topic, data_class=PointCloud,
                                                   callback=self.handleNewFrame, queue_size=1)
        self._window_size = 5
        self._sliding_window = Queue(maxsize=self._window_size)
        header = ['timestamp', 'features_0_4']
        self._log_file = open(file=log_filename, mode='w')
        self._csv_writer = csv.writer(self._log_file, delimiter=',')
        self._csv_writer.writerow(header)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self) -> None:
        if not self._log_file.closed:
            self._log_file.close()

    def handleNewFrame(self, msg: PointCloud) -> None:

        if self._sliding_window.qsize() == self._window_size:
            self._sliding_window.get_nowait()
        newest_ids_set = set()
        for i in range(len(msg.points)):
            id_float = msg.channels[0].values[i] + 0.5
            id = int(id_float)
            newest_ids_set.add(id)
        self._sliding_window.put(newest_ids_set)

        if self._sliding_window.qsize() < self._window_size:
            rospy.loginfo(f'Waiting for sliding window to be filled with data! {self._sliding_window.qsize()=}')
            return

        # for i in range(self._window_size-1):
            # rospy.loginfo(
            #   f'between 0 and {i+1}: {len(self._sliding_window.queue[0].intersection(self._sliding_window.queue[i+1]))}')
        # rospy.loginfo('----------')
        self._csv_writer.writerow([msg.header.stamp.to_sec(), len(
            self._sliding_window.queue[0].intersection(self._sliding_window.queue[4]))])


if __name__ == '__main__':
    rospy.init_node('feature_tracker_evaluation')
    log_filename = rospy.get_param('~log_filename')
    evaluator = Evaluator(cloud_topic='/feature_cloud', log_filename=log_filename)

    rospy.spin()
