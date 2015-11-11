#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
	print msg.height, msg.width, len(msg.data)


if __name__ == "__main__":
	rospy.init_node("test_pointcloud")
	rospy.Subscriber("/stereo_forward/points2", PointCloud2, callback)

	print "Success"

	rospy.spin()