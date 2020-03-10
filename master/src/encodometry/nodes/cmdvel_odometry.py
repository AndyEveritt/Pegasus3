#!/usr/bin/env python
import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from src.pose import Pose


class odometry_node:

    def __init__(self):
        self.pose = Pose()
        self.lastTime = 0

    def setTime(self, newTime):
        self.lastTime = newTime

    def main(self):
        self.odomPub = rospy.Publisher(
            'odometry/wheel', Odometry, queue_size=10)
        self.tfPub = TransformBroadcaster()

        rospy.init_node('odometry_node')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        self.rate = float(rospy.get_param('~rate', 10.0))
        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_link')
        self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')

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
        deltaX = (self.pose.xVel * cos(self.pose.theta) - self.pose.yVel * sin(self.pose.theta)) * deltaTime
        deltaY = (self.pose.xVel * sin(self.pose.theta) + self.pose.yVel * cos(self.pose.theta)) * deltaTime
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
        self.tfPub.sendTransform((self.pose.x, self.pose.y, 0), (q.x, q.y, q.z, q.w),
                                 now, self.baseFrameID, self.odomFrameID)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = q
        odom.pose.covariance[0] = 2
        odom.pose.covariance[7] = 2
        odom.pose.covariance[14] = 100000
        odom.pose.covariance[21] = 100000
        odom.pose.covariance[28] = 100000
        odom.pose.covariance[35] = 1

        odom.twist.twist.linear.x = self.pose.xVel
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.pose.thetaVel
        odom.twist.covariance[0] = 1
        odom.twist.covariance[7] = 1
        odom.twist.covariance[14] = 100000
        odom.twist.covariance[21] = 100000
        odom.twist.covariance[28] = 100000
        odom.twist.covariance[35] = 1
        self.odomPub.publish(odom)

    def __getitem__(self, item):
        return getattr(self, item)


if __name__ == '__main__':
    try:
        node = odometry_node()
        node.main()
    except rospy.ROSInterruptException:
        pass
