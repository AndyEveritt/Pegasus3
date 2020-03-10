#!/usr/bin/env python
import numpy as np
import utm
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class initialMap:

    def main(self):
        rospy.init_node('set_initial_map')
        self.initialPosePub = rospy.Publisher('set_pose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("{0} started".format(rospy.get_name()))
        rospy.Subscriber("local_xy_origin", PoseStamped, callback=self.callback)
        rospy.spin()
        pass

    def callback(self, msg):
        initialUtm = utm.from_latlon(msg.pose.position.y, msg.pose.position.x)
        mapUtm = rospy.get_param('~mapUtm', "517819.8383267763 4249616.747543993 12 S")
        mapUtm = mapUtm.split()

        initialPose = PoseWithCovarianceStamped()
        initialPose.header.stamp = rospy.get_rostime()
        initialPose.header.frame_id = "map"
        initialPose.pose.pose.position.x = initialUtm[0] - float(mapUtm[0])
        initialPose.pose.pose.position.y = initialUtm[1] - float(mapUtm[1])
        initialPose.pose.pose.position.z = 0.0
        initialPose.pose.pose.orientation.w = 1.0
        initialPose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        self.initialPosePub.publish(initialPose)
        rospy.loginfo('set initial pose {} {}'.format(initialPose.pose.pose.position.x, initialPose.pose.pose.position.y))


if __name__ == '__main__':
    try:
        initialMap = initialMap()
        initialMap.main()
    except rospy.ROSInterruptException:
        pass
