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
    if(type(data).__name__ == 'Twist'):
        linear_velocity = int(100 + (100 * message.linear.x))
        angular_velocity = int(100 + (100 * message.angular.z))
        serial_conn.write(struct.pack('>B', 253))
        serial_conn.write(struct.pack('>B', linear_velocity))
        serial_conn.write(struct.pack('>B', angular_velocity))

    elif(type(data.__name) == 'ArmCommand'):
        base_rotation_velocity = int(100 + (100 * message.base_rotation_velocity))
        arm_actuator_1_velocity = int(100 + (100 * message.arm_actuator_1_velocity))
        arm_actuator_2_velocity = int(100 + (100 * message.arm_actuator_2_velocity))
        wrist_rotation_velocity = int(100 + (100 * message.wrist_rotation_velocity))
        wrist_actuator_velocity = int(100 + (100 * message.wrist_actuator_velocity))
        gripper_velocity = int(100 + (100 * message.gripper_velocity))
        serial_conn.write(struct.pack('>B', 254)) # New message byte
        serial_conn.write(struct.pack('>B', base_rotation_velocity))
        serial_conn.write(struct.pack('>B', arm_actuator_1_velocity))
        serial_conn.write(struct.pack('>B', arm_actuator_2_velocity))
        serial_conn.write(struct.pack('>B', wrist_rotation_velocity))
        serial_conn.write(struct.pack('>B', wrist_actuator_velocity))
        serial_conn.write(struct.pack('>B', gripper_velocity))



def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rover_target_vel", Twist, callback)
    rospy.Subscriber("rover_arm_command", ArmCommand, callback)

    rate = rospy.Rate(20)

    while(True):
        rate.sleep()

if __name__ == "__main__":
    main()
