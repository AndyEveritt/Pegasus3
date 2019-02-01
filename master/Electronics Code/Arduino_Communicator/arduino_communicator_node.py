#!/usr/bin/env python

import rospy
import serial
import struct

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from controller_interpreter.msg import ArmCommand

serial_conn = serial.Serial('/dev/ttyACM0', 9600, timeout=1, writeTimeout=1)
drive_message = Twist()
arm_message = ArmCommand

# there is a way to integrate both callback functions, unclear how for now
def drive_callback(data):
    drive_message = data
    message_items = [a for a in dir(drive_message) if not a.startswith('__') and not callable(getattr(drive_message,a))]
    serial_conn.write(struct.pack('>B', 253)) # New message byte
    for message_item in message_items:
        message_value = int(100 + 100 * getattr(drive_message, message_item))
        serial_conn.write(struct.pack('>B', message_value))

def arm_callback(data):
    arm_message = data
    message_items = [a for a in dir(arm_message) if not a.startswith('__') and not callable(getattr(arm_message,a))]
    serial_conn.write(struct.pack('>B', 254)) # New message byte
    for message_item in message_items:
        message_value = int(100 + 100 * getattr(arm_message, message_item))
        serial_conn.write(struct.pack('>B', message_value))


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rover_target_vel", Twist, drive_callback)
    rospy.Subscriber("rover_arm_command", ArmCommand, arm_callback)

    rate = rospy.Rate(20)

    while(True):
        rate.sleep()

if __name__ == "__main__":
    main()
