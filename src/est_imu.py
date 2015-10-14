#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy
import time
from tf import transformations as trans


class AnkleRobot:

    def __init__(self):
        self.linear_velocity = numpy.array([0, 0, 0])
        self.position = numpy.array([0, 0, 0])
        self.last_time = time.time()
        self.test = 0

    def process_imu(self, msg):
        orient, angvel, linacc = self.unpack_imu(msg)

        t = time.time()
        dt = t - self.last_time

        # -- Change the coordinate system using msg.orientation
        # -- Force velocity to zero on heel strike
        self.linear_velocity = self.linear_velocity + linacc * dt
        self.position = self.position + self.linear_velocity * dt
        self.last_time = t

    def unpack_imu(self, msg):
        orientation = numpy.array([msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z,
                                   msg.orientation.w])
        angular_velocity = numpy.array([msg.angular_velocity.x,
                                        msg.angular_velocity.y,
                                        msg.angular_velocity.z])
        linear_acceleration = numpy.array([msg.linear_acceleration.x,
                                           msg.linear_acceleration.y,
                                           msg.linear_acceleration.z])
        return orientation, angular_velocity, linear_acceleration


def main():
    rospy.init_node("est_imu")

    ar = AnkleRobot()
    rospy.Subscriber('/imu/data', sensor_msgs.msg.Imu, ar.process_imu)
    print "Running"

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
    	print ar.position
        rate.sleep()


if __name__ == '__main__':
    main()
