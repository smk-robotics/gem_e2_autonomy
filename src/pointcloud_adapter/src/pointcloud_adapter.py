#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

class PointCloudAdapterNode():
    def __init__(self):
      self._output_frame_id      = rospy.get_param("~output_frame_id")
      self._pointcloud_subsriber = rospy.Subscriber(
          "input/pointcloud", PointCloud2, self.__pointcloud_callback
      )
      self._pointcloud_publisher = rospy.Publisher(
          "output/pointcloud", PointCloud2, queue_size=1
      )

    def __pointcloud_callback(self, msg):
        msg.header.frame_id = self._output_frame_id
        self._pointcloud_publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_adapter_node")
    node = PointCloudAdapterNode()
    rospy.spin()