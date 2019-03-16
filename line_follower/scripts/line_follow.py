#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class LineFollower(object)
    def __init__(self):
        self.bridge = CvBridge()
        #placeholder
        #self.image_sub = rospy.Subscriber("/vid_feed", Image, self.camera_callback)
        # dictionary of 4 cardinal directions and one stop for line follow task
        self.right = Twist()
        self.left = Twist()
        self.up = Twist()
        self.down = Twist()
        self.stop = Twist()

        self.right.linear.x = 0.5
        self.left.linear.x = -0.5
        self.up.linear.z = 0.5
        self.down.linear.z = -0.5

        self.directions = {'up' : self.up, 'down' : self.down, 'left' : self.left, 'stop' : self.stop}

    
    def makeDirectionMessage(self, direction):
        return self.directions.get('direction', self.stop)

    def camera_callback(self, data):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # downsampling to 212 x 80 to speed things up
        newsize = (212, 160)
        interpolation = cv2.INTER_NEAREST
        resized_img = cv2.resize(image, newsize, 0, 0, interpolation)

        # convert to hsv for better color thresholding 
        hsv = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)

        # filtering image to red and not red since only those colors matter
        # since our hue values wrap from 330 to 30, but that's not how being
        # in between numbers works, we gotta 
        lower_red = np.array([330, 60, 60])
        inbw_red1 = np.array([0, 255, 255])
        inbw_red2 = np.array([0, 60, 60])
        upper_red = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, inbw_red1)
        mask2 = cv2.inRange(hsv, inbw_red2, upper_red)
        mask = mask1 + mask2

        # find centroid which will be used to keep line-following on track 
        # and decide when to change direction and what direction to change too
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        # choosing direction
        
        direction = self.makeDirectionMessage()

