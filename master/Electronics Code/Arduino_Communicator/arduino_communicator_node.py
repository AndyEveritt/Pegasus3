#!/usr/bin/env python

import rospy
import serial
import struct

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy

serial_conn = serial.Serial('/dev/ttyACM0', 9600, timeout=1, writeTimeout=1)
message = Twist()

def callback(data):
    message = data
    linear_velocity = int(100 + (100 * message.linear.x))
    angular_velocity = int(100 + (100 * message.angular.z))
    serial_conn.write(struct.pack('>B', 254))
    serial_conn.write(struct.pack('>B', linear_velocity))
    serial_conn.write(struct.pack('>B', angular_velocity))

def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rover_target_vel", Twist, callback)

    rate = rospy.Rate(20)

    while(True):
        rate.sleep()

if __name__ == "__main__":
    main()
