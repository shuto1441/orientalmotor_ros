#!/usr/bin/env python

import rospy
from orientalmotor_ros import MotorReconfigure

def main():
    rospy.init_node("motor_control")
    sub = MotorReconfigure.OrientalMotor()

    rospy.spin()

if __name__ == '__main__':
    main()
