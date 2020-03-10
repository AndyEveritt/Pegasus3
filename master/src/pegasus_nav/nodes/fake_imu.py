#!/usr/bin/env python
import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xVel = 0
        self.yVel = 0
        self.thetaVel = 0

    def __str__(self):
        return str({'x': self.x, 'y': self.y, 'theta': self.theta,
                    'xVel': self.xVel, 'yVel': self.yVel, 'thetaVel': self.thetaVel})


class odometry_node:

    def __init__(self):
        self.pose = Pose()
        self.lastTime = 0

    def setTime(self, newTime):
        self.lastTime = newTime

    def main(self):
        self.imuPub = rospy.Publisher(
            'imu/data', Imu, queue_size=10)

        rospy.init_node('fake_imu')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        self.rate = float(rospy.get_param('~rate', 10.0))
        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_link')

        self.setTime(rospy.get_time())
        rate = rospy.Rate(self.rate)
        rospy.Subscriber("rover_target_vel", Twist,
                         callback=self.callback)
        rospy.spin()

    def callback(self, data):
        self.calculate_pose(data)

    def calculate_pose(self, data):
        # time stuff
        newTime = rospy.get_time()
        deltaTime = newTime - self.lastTime
        self.setTime(newTime)

        # set poses
        self.pose.xVel = data.linear.x
        self.pose.yVel = data.linear.y
        self.pose.thetaVel = data.angular.z
        deltaX = (self.pose.xVel * cos(self.pose.theta) -
                  self.pose.yVel * sin(self.pose.theta)) * deltaTime
        deltaY = (self.pose.xVel * sin(self.pose.theta) +
                  self.pose.yVel * cos(self.pose.theta)) * deltaTime
        deltaTh = self.pose.thetaVel * deltaTime
        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = (self.pose.theta + deltaTh) % (2*pi)

        self.lastTime = newTime

        now = rospy.get_rostime()

        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = sin(self.pose.theta / 2)
        q.w = cos(self.pose.theta / 2)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.baseFrameID
        imu.orientation = q
        imu.orientation_covariance = [1e9, 1e9,
            1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9]

        imu.angular_velocity.x = 0
        imu.angular_velocity.y = 0
        imu.angular_velocity.z = self.pose.thetaVel
        imu.angular_velocity_covariance = [
            1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9]


        imu.linear_acceleration.x = 0
        imu.linear_acceleration.y = 0
        imu.linear_acceleration.z = 0
        imu.linear_acceleration_covariance = [
            1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9]

        self.imuPub.publish(imu)

    def __getitem__(self, item):
        return getattr(self, item)


if __name__ == '__main__':
    try:
        node = odometry_node()
        node.main()
    except rospy.ROSInterruptException:
        pass
