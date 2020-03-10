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

def callback(data):
    message = data
    message_items = [a for a in dir(message) if not a.startswith('__') and not callable(getattr(message,a))]
    if(type(data).__name__ == 'Twist'):
        serial_conn.write(struct.pack('>B', 253)) # New message byte
    elif(type(data.__name) == 'ArmCommand'):
        serial_conn.write(struct.pack('>B', 254)) # New message byte
    for message_item in message_items:
        message_value = int(100 + 100 * getattr(message, message_item))
        serial_conn.write(struct.pack('>B', message_value))


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rover_target_vel", Twist, callback)
    rospy.Subscriber("rover_arm_command", ArmCommand, callback)

    rate = rospy.Rate(20)

    while(True):
        rate.sleep()

if __name__ == "__main__":
    main()
