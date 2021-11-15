#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from orientalmotor_ros.cfg import modbus_motorConfig
from orientalmotor_ros.msg import motor
from rospy import client
import serial
import time
import tkinter as tk

class modbus_ros():
    def __init__(self):
        #通信の接続
        self.client = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
        self.size = 16
        self.pulse_angle= 0.36
        print(self.client.name)
        self.srv = Server(modbus_motorConfig, self.callback)
        self.master = tk.Tk()
        button = tk.Button(self.master, text="rotate", command=lambda: self.motor_rotate(motor))
        button.pack()
        self.master.mainloop()

    def callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {rpm}, {angle}""".format(**config))
        motor.rpm = int("{rpm}".format(**config))
        motor.angle = int("{angle}".format(**config))
        return config

    def motor_rotate(self,msg):
        rpm = msg.rpm
        angle = msg.angle
        self.apply_operation_method()
        time.sleep(0.1)
        self.apply_angle(angle)
        time.sleep(0.1)
        self.apply_rpm(rpm)
        time.sleep(0.1)
        self.start()
        time.sleep(0.1)
        self.stop()

    #運転データNo.0の運転方式
    def apply_operation_method(self):
        command = b"\x01\x06\x05\x01\x00\x00\xd8\xc6"
        self.client.write(command)
        result = self.client.read(self.size)
        print("operation: {}".format(result))

    def error_check(self,query):
        crc_register = 0xFFFF
        for data_byte in query:
            crc_register ^= data_byte
            for _ in range(8):
                overflow = crc_register & 1 == 1
                crc_register >>= 1
                if overflow:
                    crc_register ^= 0xA001
        # 結果は(上位→下位)の順
        return crc_register.to_bytes(2, 'little')

    def rpm_to_bytes(self,rpm):
        hz = int(rpm/60*360/self.pulse_angle)
        command = hz.to_bytes(2,byteorder='big')
        return command

    def angle_to_bytes(self,angle):
        step = int(-angle/self.pulse_angle)
        command = step.to_bytes(4,byteorder='big', signed=True)
        return command

    def apply_angle(self,angle):
        command = b"\x01\x10\x04\x00\x00\x02\x04" + self.angle_to_bytes(angle)
        command += self.error_check(command)
        self.client.write(command)
        result = self.client.read(self.size)
        print("step set: {}".format(result))

    def apply_rpm(self,rpm):
        command = b"\x01\x10\x04\x80\x00\x02\x04\x00\x00" + self.rpm_to_bytes(rpm)
        command += self.error_check(command)
        self.client.write(command)
        result = self.client.read(self.size)
        print("rpm set: {}".format(result))

    #START入力 ON （運転No.0 運転開始）
    def start(self):
        command = b"\x01\x06\x00\x7d\x00\x08\x18\x14"
        self.client.write(command)
        result = self.client.read(self.size)
        print("start on: {}".format(result))

    #START入力 OFF （運転No.0 運転終了）
    def stop(self):
        command = b"\x01\x06\x00\x7d\x00\x00\x19\xd2"
        self.client.write(command)
        result = self.client.read(self.size)
        print("start off: {}".format(result))


if __name__ == "__main__":
    rospy.init_node("orientalmotor_ros", anonymous = True)
    motor_controller = modbus_ros()
    rospy.spin()
