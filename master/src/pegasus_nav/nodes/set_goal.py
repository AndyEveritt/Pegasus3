#!/usr/bin/env python
import rospy
import utm
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


class setGoal:

    def __init__(self):
        self.recievedGPS = False

    def main(self):
        rospy.init_node('set_goal')
        self.setGoalPub = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=5)
        rospy.Subscriber('gps/filtered', NavSatFix, callback=self.updateGPS)
        # if self.recievedGPS:
        #     self.calcUTMdiff()
        rospy.spin()

    def calcUTMdiff(self):
        goal = PoseStamped()
        goal.header.frame_id = 'base_link'

        isGoalDMS = rospy.get_param('~isGoalDMS', False)
        goalLat = rospy.get_param('~goalLat', 38.372)
        goalLon = rospy.get_param('~goalLon', -110.70453)
        if isGoalDMS:
            self.goalLatDD = self.parse_dms(goalLat)
            self.goalLonDD = self.parse_dms(goalLon)
            self.goalUtm = self.dd2utm(self.goalLatDD, self.goalLonDD)
        else:
            self.goalUtm = self.dd2utm(goalLat, goalLon)
        
        goal.pose.position.x = self.goalUtm[0] - self.roverUtm[0]
        goal.pose.position.y = self.goalUtm[1] - self.roverUtm[1]
        goal.pose.orientation.w = 1

        self.setGoalPub.publish(goal)

    def updateGPS(self, msg):
        self.roverLatDD = msg.latitude
        self.roverLonDD = msg.longitude
        self.roverUtm = self.dd2utm(self.roverLatDD, self.roverLonDD)
        self.recievedGPS = True
        self.calcUTMdiff()

    def dms2dd(self, degrees, minutes, seconds, direction):
        dd = float(degrees) + float(minutes)/60 + float(seconds)/(60*60)
        if direction == 'W' or direction == 'S':
            dd *= -1
        return dd

    def dd2dms(self, deg):
        d = int(deg)
        md = abs(deg - d) * 60
        m = int(md)
        sd = (md - m) * 60
        return [d, m, sd]

    def parse_dms(self, dms):
        parts = dms.split(' ')
        lat = self.dms2dd(parts[0], parts[1], parts[2], parts[3])

        return (lat)

    def dd2utm(self, lat, lon):
        return utm.from_latlon(lat, lon)


if __name__ == '__main__':
    try:
        setGoal0 = setGoal()
        setGoal0.main()
        # setGoal0.calcUTMdiff()
    except rospy.ROSInterruptException:
        pass
