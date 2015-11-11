#!/usr/bin/env python

import sys

import numpy as np
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud

class CloudStitcher(object):
    def __init__(self):
        self.tf = tf.TransformListener()
        self.seq = 0

        # subscribe to point clouds
        self.cloud_sub = rospy.Subscriber('/stereo/points2', PointCloud2, self._cloud_cb, queue_size=5)

        # output publisher
        self.cloud_pub = rospy.Publisher('/fused_points', PointCloud2, queue_size=5)
        

    def _cloud_cb(self, cloud):
        points = np.array(list(read_points(cloud, skip_nans=True)))
        if points.shape[0] == 0:
            return
        
        pos = points[:,0:3]
        cor = np.reshape(points[:,-1], (points.shape[0], 1))

        # Get 4x4 matrix which transforms point cloud co-ords to odometry frame
        try:
            points_to_map = self.tf.asMatrix('/odom', cloud.header)
        except tf.ExtrapolationException:
            return

        transformed_points = points_to_map.dot(np.vstack((pos.T, np.ones((1, pos.shape[0])))))
        transformed_points = transformed_points[:3,:].T

        self.seq += 1
        header = Header()
        header.seq = self.seq
        header.stamp = rospy.Time.now()
        header.frame_id = '/odom'

        output_cloud = create_cloud(header, cloud.fields, np.hstack((transformed_points, cor)))
        self.cloud_pub.publish(output_cloud)

def main():
    rospy.init_node('pointcloud_accumulate')
    stitcher = CloudStitcher()
    rospy.spin()

if __name__ == '__main__':
    main()