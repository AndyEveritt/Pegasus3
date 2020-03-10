#!/usr/bin/env python

import rospy
import serial
import struct

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from controller_interpreter.msg import ArmCommand


controlState = "drive"


class CommunicatorNode():
    def updateControlState(self, message):
        global controlState
        controlState = message.data

    def armCallback(self, data):
        message = data

        base_rotation_velocity = int(
            100 + (100 * message.base_rotation_velocity))
        arm_actuator_1_velocity = int(
            100 + (100 * message.arm_actuator_1_velocity))
        arm_actuator_2_velocity = int(
            100 + (100 * message.arm_actuator_2_velocity))
        wrist_rotation_velocity = int(
            100 + (100 * message.wrist_rotation_velocity))
        wrist_actuator_velocity = int(
            100 + (100 * message.wrist_actuator_velocity))
        gripper_velocity = int(100 + (100 * message.gripper_velocity))

        arm_msg = Int16MultiArray()
        arm_msg.data = [base_rotation_velocity,
                        arm_actuator_1_velocity,
                        arm_actuator_2_velocity,
                        wrist_rotation_velocity,
                        wrist_actuator_velocity,
                        gripper_velocity]

        self.arm_pub.publish(arm_msg)

    def driveCallback(self, data):
        message = data

        linear_velocity = int(100 + (100 * message.linear.x))
        angular_velocity = int(100 + (100 * message.angular.z))

        drive_msg = Int16MultiArray()
        drive_msg.data = [linear_velocity,
                            angular_velocity]

        self.drive_pub.publish(drive_msg)

    def main(self):
        rospy.init_node('arduino_communicator', anonymous=True)
        rospy.Subscriber("control_state", String, self.updateControlState)
        rospy.Subscriber("rover_target_vel", Twist, self.driveCallback)
        rospy.Subscriber("rover_arm_command", ArmCommand, self.armCallback)

        self.drive_pub = rospy.Publisher(
            "arduino/drive_command", Int16MultiArray, queue_size=5)
        self.arm_pub = rospy.Publisher("arduino/arm_command",
                                       Int16MultiArray, queue_size=5)

        rate = rospy.Rate(20)

        while(True):
            rate.sleep()


if __name__ == "__main__":
    communicator = CommunicatorNode()
    communicator.main()
    rospy.spin()
