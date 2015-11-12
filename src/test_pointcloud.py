#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
import numpy as np

def callback(msg):
	points = np.array(list(read_points(msg, skip_nans=True)))
	print msg.height, msg.width, len(msg.data), points


if __name__ == "__main__":
	rospy.init_node("test_pointcloud")
	rospy.Subscriber("/normal_estimation/output", PointCloud2, callback)

	print "Success"

	rospy.spin()
