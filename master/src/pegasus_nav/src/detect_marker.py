#!/usr/bin/env python

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import imutils
import tf
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from actionlib_msgs.msg import GoalID
from multiScale_templateMatching import getTemplate, multiScaleTemplateMatching


class Detector():

    # ROS node that detects tennis balls (or any object) using multiscale
    # template matching to gather evidence, with a blob detector used to identify
    # possible matches.
    # Calculates and publishes the position of detected markers

    def __init__(self):

        # initialize ROS node and transform publisher
        rospy.init_node('marker_detector', anonymous=True)

        self.rate = rospy.Rate(50.0)    # publish transform at 50 Hz

        # initialize values for marker location within camera image
        self.centre_x = 0       # x is pixels left(0) to right(+)
        self.centre_y = 0       # y is pxels top(0) to bottom(-)
        self.distance = 0       # distance from camera to marker from triangle similarity
        self.last_distance = 0  # last non-zero distance measurement
        self.threshold_val = "minVal"
        self.distance_constant = 96000        # marker actual width * focal length
        self.tracking = False

        self.scale_range = (rospy.get_param('scale_upper', default=0.5),
                            rospy.get_param('scale_lower', default=1),
                            rospy.get_param('scale_steps', default=2))

        # Instructions
        print "Press 's' key when there is no tennis ball present"
        print "Press the 'r' key to reset"

        # convert image from a ROS image message to a CV image
        self.bridge = CvBridge()

        # wait for the camera_info topic to become available
        # rospy.loginfo("Waiting for usb_cam message.")
        # rospy.wait_for_message('/usb_cam/image_raw', CameraInfo)
        # rospy.loginfo("usb_cam message received.")

        # subscribe to camera_info to get image frame height and width
        rospy.Subscriber('/usb_cam/camera_info', CameraInfo,
                         self.camera_data, queue_size=1)

        rospy.Subscriber('/explore_server/explore_costmap/explore_boundary/frontiers',
                         PointCloud2, self.set_threshold, queue_size=1)
        self.pub_explore_cancel = rospy.Publisher(
            'explore_server/cancel', GoalID, queue_size=1)

        # Subscribe to camera images
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.image_callback, queue_size=1)

        self.image_pub = rospy.Publisher(
            "/usb_cam/tracker_results", Image, queue_size=1)

        # Initialise publisher for goals
        self.pub_goal = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=1)

        # Initialise publisher for target velocity
        self.pub_cmd_vel = rospy.Publisher(
            'rover_target_vel', Twist, queue_size=1)

        # Initialise publisher for target velocity
        self.pub_stopped = rospy.Publisher(
            'marker_found', Bool, queue_size=1)

    # This callback function sets parameters regarding the camera.
    def camera_data(self, data):
        # set values on the parameter server
        # rot_camera_frame_2
        rospy.set_param('mast_cam_link', data.header.frame_id)
        rospy.set_param('mast_cam_height', data.height)         # height is 720
        rospy.set_param('mast_cam_width', data.width)           # width is 1280

        # set values for local variables
        self.cam_height = data.height
        self.cam_width = data.width
        self.cam_fov = 60

    def set_threshold(self, msg):
        rospy.set_param('set_tracker', True)

    # This callback function handles processing images looking for markers
    def image_callback(self, msg):
        # convert ROS image to OpenCV image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            print(error)

        # get HSV template of tennis ball
        filename = sys.path[0] + \
            rospy.get_param('template', default='/tennisBallTemplate.jpg')
        tH, tW, hsv_template = getTemplate(filename, width=24)

        # global variables

        self.frame_width = rospy.get_param('frame_width', 480)

        key = cv2.waitKey(1) & 0xFF

        # resize image to a known width but retain aspect ratio
        image = imutils.resize(image, width=self.frame_width)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Do template matching
        accumulator, ball_locations, (minVal, _, _) = multiScaleTemplateMatching(
            display_image=image, image=hsv_image, template=hsv_template,
            scale_range=self.scale_range, threshold=self.threshold_val)

        if key == ord("s"):
            rospy.set_param('set_tracker', True)
        elif key == ord("r"):
            rospy.set_param('reset_tracker', True)

        if rospy.get_param('set_tracker', default=False):
            # press when no ball is present to set upper limit
            self.threshold_val = str(minVal*rospy.get_param('threshold_mulitplier', default=0.7))
            self.tracking = True
            print "Tennis ball threshold value = %s" % self.threshold_val
            print "No ball threshold value = %f" % minVal
            rospy.set_param('set_tracker', False)

        elif rospy.get_param('reset_tracker', default=False):
            # reset threshold value
            self.threshold_val = "minVal"
            self.tracking = False
            print "reset threshold"
            rospy.set_param('reset_tracker', False)

        if self.tracking:
            cv2.circle(image, (5, 5), 5, (0, 255, 255), -1)
            cv2.putText(image, "%s" % self.threshold_val, (5, 40),
                        5, 1, (255, 0, 0), 1, cv2.LINE_AA)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        try:
            cv2.putText(image, "%d" %
                        len(accumulator.keypoints), (20, 20), 5, 1, (0, 255, 0), 1, cv2.LINE_AA)
            image = cv2.drawKeypoints(image, accumulator.keypoints, np.array(
                []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            self.centre_x = accumulator.keypoints[0].pt[0]
            self.centre_y = accumulator.keypoints[0].pt[1]

            # Calculate distance from contour size.
            distance_ratio = self.frame_width / float(self.cam_width)
            self.distance = self.distance_constant * distance_ratio / \
                (accumulator.keypoints[0].size * 5000)
        except:
            # print "No keypoints"
            self.centre_x = None
            self.centre_y = None
            self.distance = None

        # cv2.imshow("Image", image)

        self.find_world_coordinates(self.centre_x, self.distance)
        self.update_marker_position(self.marker_x_pos, self.marker_z_pos)
        # self.update_cmd_vel(self.centre_x)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def update_cmd_vel(self, x):
        self.marker_cmd_vel = Twist()

        self.marker_cmd_vel.linear.y = 0
        self.marker_cmd_vel.linear.z = 0
        self.marker_cmd_vel.angular.x = 0
        self.marker_cmd_vel.angular.y = 0

        self.marker_cmd_vel.linear.x = 0
        self.marker_cmd_vel.angular.z = 0
        if x != None:
            marker_x_pos = x / (self.frame_width / 2) - 1

            if (marker_x_pos < -0.5):
                self.marker_cmd_vel.linear.x = 0.0
                self.marker_cmd_vel.angular.z = 0.3
                rospy.loginfo("left")

            elif (marker_x_pos > -0.5 and marker_x_pos < 0.5):
                self.marker_cmd_vel.linear.x = 0.1
                self.marker_cmd_vel.angular.z = 0
                rospy.loginfo("forward")

            elif (marker_x_pos > 0.5):
                self.marker_cmd_vel.linear.x = 0.0
                self.marker_cmd_vel.angular.z = -0.3
                rospy.loginfo("right")

        self.pub_cmd_vel.publish(self.marker_cmd_vel)

    # This function publishes the marker goal position.
    def update_marker_position(self, x, z):
        
        marker_found = Bool()

        if (x != None) & (z != None):
            # cancel exploration
            cancel = GoalID()
            marker_found.data = True
            self.pub_explore_cancel.publish(cancel)
            self.pub_stopped.publish(marker_found)
            print "Cancelled Exploration"

            # send goal using position of marker.
            self.marker_goal = PoseStamped()
            self.marker_goal.header.frame_id = 'base_link'
            self.marker_goal.pose.position.x = z
            self.marker_goal.pose.position.y = -x

            self.marker_goal.pose.orientation.w = 1.0

            # self.pub_goal.publish(self.marker_goal)

        else:
            marker_found = False
            self.pub_stopped.publish(marker_found)
            
        # rospy.loginfo("New goal:  %f %f", x, y)

    # Calculates the real world coordinates of the marker
    def find_world_coordinates(self, marker_x, distance):

        if (marker_x != None) & (distance != None):

            # X co-ordinate
            x_from_centre = marker_x - self.frame_width / 2

            # Angle from camera facing direction
            x_ratio = x_from_centre/(self.frame_width / 2)
            d0 = 1/np.sin(math.radians(self.cam_fov / 2))
            z0 = math.sqrt(d0**2 - 1)
            theta = math.atan(x_ratio/z0)

            # marker co-ordinates using camera axis
            self.marker_x_pos = self.distance * np.sin(theta)
            self.marker_z_pos = self.distance * np.cos(theta)
        else:
            self.marker_x_pos = None
            self.marker_z_pos = None


if __name__ == '__main__':

    # start up the detector node and run until shutdown by interrupt
    try:
        detector = Detector()
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord("s"):
        #     # press when no ball is present to set upper limit
        #     threshold_val = str(minVal*0.75)
        #     print "Tennis ball threshold value = %s" % threshold_val
        #     print "No ball threshold value = %f" % minVal

        # elif key == ord("r"):
        #     #reset threshold value
        #     threshold_val = "minVal"
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")

    # close all terminal windows when process is shut down
    cv2.destroyAllWindows()
