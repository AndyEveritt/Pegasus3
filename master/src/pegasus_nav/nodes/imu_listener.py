#!/usr/bin/env python
import rospy
from math import pi, sin, cos, radians
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Imu


class imu_node:

    def callback(self, data):
        self.OX = -radians(
            float(data.data[data.data.index("A")+1:data.data.index("B")]))
        self.OY = -radians(
            float(data.data[data.data.index("B")+1:data.data.index("C")]))
        self.OZ = -radians(
            float(data.data[data.data.index("C")+1:data.data.index("D")]))
        self.LX = radians(
            float(data.data[data.data.index("D")+1:data.data.index("E")]))
        self.LY = radians(
            float(data.data[data.data.index("E")+1:data.data.index("F")]))
        self.LZ = radians(
            float(data.data[data.data.index("F")+1:data.data.index("G")]))
        self.AX = radians(
            float(data.data[data.data.index("G")+1:data.data.index("H")]))
        self.AY = radians(
            float(data.data[data.data.index("H")+1:data.data.index("I")]))
        self.AZ = radians(
            float(data.data[data.data.index("I")+1:data.data.index("J")]))

        imu = Imu()
        now = rospy.get_rostime()

        cy = cos(self.OZ * 0.5)
        sy = sin(self.OZ * 0.5)
        cp = cos(self.OY * 0.5)
        sp = sin(self.OY * 0.5)
        cr = cos(self.OX * 0.5)
        sr = sin(self.OX * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.baseFrameID
        imu.orientation = q
        imu.orientation_covariance = [
            10, 1e9, 1e9, 1e9, 1e9, 10, 1e9, 1e9, 0.01]

        imu.angular_velocity.x = self.AX
        imu.angular_velocity.y = self.AY
        imu.angular_velocity.z = self.AZ
        imu.angular_velocity_covariance = [
            1, 1e9, 1e9, 1e9, 1, 1e9, 1e9, 1e9, 0.1]

        imu.linear_acceleration.x = self.LX
        imu.linear_acceleration.y = self.LY
        imu.linear_acceleration.z = self.LZ
        imu.linear_acceleration_covariance = [
            1, 1e9, 1e9, 1e9, 1, 1e9, 1e9, 1e9, 1]

        self.imuPub.publish(imu)

    def main(self):
        rospy.init_node('imu_listener', anonymous=True)
        self.imuPub = rospy.Publisher("imu/data", Imu, queue_size=10)
        self.baseFrameID = rospy.get_param('~base_frame_id', 'imu')

        rospy.Subscriber("imu/raw", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == "__main__":
    try:
        node = imu_node()
        node.main()
    except rospy.ROSInterruptException:
        pass
