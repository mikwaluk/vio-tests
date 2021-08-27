#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class OdomPathPublisher:
    def __init__(self, color: ColorRGBA) -> None:
        publish_topic = '/ground_truth_path'  # '/ground_truth_wheel_odom_path'
        odom_topic = '/odom'  # '/sensors/can_odom'
        self._odom_path_pub = rospy.Publisher(publish_topic, MarkerArray, queue_size=1)
        self._markers_count = 0
        self._last_publishing_time = rospy.Duration()
        self._can_odom_sub = rospy.Subscriber(name=odom_topic,
                                              data_class=Odometry,
                                              callback=self.publishOdomPath)
        self._marker_color = color

    def publishOdomPath(self, msg: Odometry) -> None:
        if (rospy.Time.now() - self._last_publishing_time).to_nsec() < rospy.Duration(0.02).to_nsec():
            return

        new_pose_marker = Marker()
        new_pose_marker.id = self._markers_count
        new_pose_marker.action = Marker.ADD
        new_pose_marker.type = Marker.SPHERE
        new_pose_marker.scale.x = 0.5
        new_pose_marker.scale.y = 0.5
        new_pose_marker.scale.z = 0.5
        new_pose_marker.color = self._marker_color
        new_pose_marker.header.frame_id = 'world'
        new_pose_marker.header.stamp = msg.header.stamp
        new_pose_marker.pose = msg.pose.pose

        path_markers = MarkerArray()
        path_markers.markers = [new_pose_marker]

        self._markers_count += 1
        self._last_publishing_time = rospy.Time.now()
        self._odom_path_pub.publish(path_markers)


if __name__ == '__main__':
    rospy.init_node('odom_path_publisher')

    r = rospy.get_param('~r', 1.0)
    g = rospy.get_param('~g', 0.0)
    b = rospy.get_param('~b', 0.0)
    marker_color = ColorRGBA(r=r, g=g, b=b, a=1.0)

    publisher = OdomPathPublisher(color=marker_color)

    rospy.spin()
