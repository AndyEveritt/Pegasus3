import rospy
from geometry_msgs.msg import Twist


class filter_node:
    def __init__(self):
        self.filterNum = 5  # how many previous velocity commands to filter

    def main(self):
        rospy.init_node('filter_vel')

        self.velPub = rospy.Publisher('rover_target_vel', Twist, queue_size=10)
        self.prevVels = []

        for i in range(self.filterNum):
            self.prevVels.append(Twist())

        rospy.Subscriber('autonomous_target_vel',
                         Twist, callback=self.callback)
        rospy.spin()

    def callback(self, data):
        self.storeData(data)

    def storeData(self, data):
        self.prevVels.pop(0)
        self.prevVels.append(data)

    def filterData(self):
        maxLinX = 0
        maxAngZ = 0
        for i in range(self.filterNum):
            if (self.prevVels[i].Twist.linear.x > maxLinX):
                maxLinX = self.prevVels[i].Twist.linear.x
            if (self.prevVels[i].Twist.angular.z > maxAngZ):
                maxLinX = self.prevVels[i].Twist.angular.z
        self.filterVel = Twist()
        self.filterVel.linear = {maxLinX, 0, 0}
        self.filterVel.angular = {0, 0, maxAngZ}

if __name__ == '__main__':
    try:
        node = filter_node()
        node.main()
    except rospy.ROSInterruptException:
        pass
