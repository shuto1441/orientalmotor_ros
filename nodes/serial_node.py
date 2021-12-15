#!/usr/bin/env python

import rospy
from orientalmotor_ros import MotorReconfigure
import sys

if __name__ == "__main__":

    rospy.init_node("orientalmotor_ros", anonymous = True)
    motor_controller = MotorReconfigure.modbus_ros()
    rospy.spin()