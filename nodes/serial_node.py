#!/usr/bin/env python

import rospy
from orientalmotor_ros import MotorReconfigure
import sys
from PyQt5.QtWidgets import QApplication

if __name__ == "__main__":

    rospy.init_node("motor_control")
    app = QApplication(sys.argv)
    ex = MotorReconfigure.OrientalMotor()
    sys.exit(app.exec_())