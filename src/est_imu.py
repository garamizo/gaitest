#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy
import time
from tf import transformations as trans
import csv


class AnkleRobot:

    def __init__(self):
        self.linear_velocity = numpy.array([0, 0, 0])
        self.position = numpy.array([0, 0, 0])
        self.time = time.time()
        self.test = 0
        self.newData = False

        self.orientation = numpy.array([0, 0, 0, 0])
        self.linear_acceleration = numpy.array([0, 0, 0])
        self.angular_velocity = numpy.array([0, 0, 0])

    def process_imu(self, msg):
        orient, angvel, linacc = self.unpack_imu(msg)

        t = time.time()
        dt = t - self.time

        # -- Change the coordinate system using msg.orientation
        # -- Force velocity to zero on heel strike
        self.linear_velocity = self.linear_velocity + linacc * dt
        self.position = self.position + self.linear_velocity * dt
        self.time = t

        self.orientation = orient
        self.angular_velocity = angvel
        self.linear_acceleration = linacc

        self.newData = True

    def process_stamped_pose(self, msg):
        t, trans, rot = self.unpack_stamped_pose(msg)

        self.mocap_trans = trans
        self.mocap_rot = rot
        self.mocap_time = t

        self.mocap_newData = True        

    def unpack_stamped_pose(self, msg):
        orientation = numpy.array([msg.transform.rotation.x,
                                   msg.transform.rotation.y,
                                   msg.transform.rotation.z,
                                   msg.transform.rotation.w])
        translation = numpy.array([msg.msg.transform.translation.x,
                                        msg.msg.transform.translation.y,
                                        msg.msg.transform.translation.z])
        time = header.stamp.secs + header.stamp.nsecs * 1e-9
        return time, translation, orientation

    def unpack_pose(self, msg):


def main():
    rospy.init_node("est_imu")

    ar = AnkleRobot()
    rospy.Subscriber('/imu/data', sensor_msgs.msg.Imu, ar.process_imu)
    print "Running"

    # rospy.sleep(1)
    # spamwriter.writerow([ar.linear_velocity[0], 1.0, True])

    # rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if ar.newData:
        	ar.newData = False
        	spamwriter.writerow([ ar.time,
        		ar.orientation[0], ar.orientation[1], ar.orientation[2], ar.orientation[3], 
        		ar.angular_velocity[0], ar.angular_velocity[1], ar.angular_velocity[2], 
        		ar.linear_acceleration[0], ar.linear_acceleration[1], ar.linear_acceleration[2]])
        	
        if ar.mocap_newData:
        	ar.mocap_newData = False
        	mocapwriter.writerow([ ar.mocap_time,
        		ar.mocap_rot[0], ar.mocap_rot[1], ar.mocap_rot[2], ar.mocap_rot[3], 
        		ar.mocap_trans[0], ar.mocap_trans[1], ar.mocap_trans[2]])




if __name__ == '__main__':
	global imuwriter, mocapwriter
	with open('imu.csv', 'wb') as csvfile1:
		with open('mocap.csv', 'wb') as csvfile2:
			imuwriter = csv.writer(csvfile, delimiter=' ')
			mocapwriter = csv.writer(csvfile, delimiter=' ')
			main()
