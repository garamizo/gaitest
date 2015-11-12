#!/usr/bin/env python

import sys

import numpy as np
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from tf import transformations

def pack_marker(pos, _xaxis):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	marker.scale.x = 3.0
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	
	# get some likely scale
	marker.color.a = 1.0
	
	# calculate q from xaxis
	xaxis = np.array(_xaxis) / np.linalg.norm(_xaxis)
	yaxis = np.cross(xaxis, [0, 1, 0])
	zaxis = np.cross(xaxis, yaxis)
	R = np.array([list(xaxis), yaxis, zaxis]).T
	T = np.hstack( (R, np.zeros((3,1))) )
	T = np.vstack( (T, np.array([0, 0, 0, 1]).reshape((1,4))) )
	q = transformations.quaternion_from_matrix(T)
	marker.pose.orientation.x = q[0]
	marker.pose.orientation.y = q[1]
	marker.pose.orientation.z = q[2]
	marker.pose.orientation.w = q[3]
	marker.pose.position.x = pos[0]
	marker.pose.position.y = pos[1]
	marker.pose.position.z = pos[2]
	
	return marker


if __name__ == "__main__":

	topic = 'visualization_marker_array'
	publisher = rospy.Publisher(topic, MarkerArray, queue_size=5)

	rospy.init_node('register')

	markerArray = MarkerArray()

	trans = [0, 0, 0]
	count = 0
	MARKERS_MAX = 10

	while not rospy.is_shutdown():

		# ... here I get the data I want to plot into a vector called trans

		marker = pack_marker([0, 0, 1], [0.5, 0, 0.5])


		# We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
		if(count > MARKERS_MAX):
			markerArray.markers.pop(0)
		else:
			count += 1
		markerArray.markers.append(marker)


		# Publish the MarkerArray
		publisher.publish(markerArray)
