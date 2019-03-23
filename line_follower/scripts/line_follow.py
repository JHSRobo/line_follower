#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        # figure out the video feed
        self.image_sub = rospy.Subscriber("/vid_feed", Image, self.camera_callback)
        self.movement_pub = rospy.Publisher("/rov/cmd_vel", Twist, 0)
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

        # axes of an image increase left to right and bottom to top
        self.y_directions = {'positive' : self.up, 'negative' : self.down}
        self.x_directions = {'positive' : self.right, 'negative' : self.left}
        self.axes = {'x' : self.x_directions, 'y' : self.y_directions, 'stop' : self.stop}
        self.direction_msg = Twist()
        # first value is the axis and the second is the direction
        self.prev_direction =['', '']
        # threshold of error (in pixels) for changing main directions
        self.threshold = 20
        self.y_err = 0
        self.x_err = 0
        self.largest_err = 0
        self.other_err = 0
        self.err_axis = ''

    def makeDirectionMessage(self, axis, error):
        try:
            self.prev_direction = [axis, error / abs(error)]
            return self.axes.get(axis).get(error / abs(error))
        except ZeroDivisionError:
            return self.axes.get(self.prev_direction[0]).get(self.prev_direction[1])

    # stopgap, should be some type of PID for best results
    def correctError(error):
        return error * error * error / (2*212*212*212)

    def keepCentered(self, error, axis):
        corrected_msg = self.direction_msg
        if axis is 'y':
            corrected_msg.linear.x += correctError(error)
        elif axis is 'x'
            corrected_msg.linear.z += correctError(error)
        return corrected_msg

    def camera_callback(self, data):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # downsampling to 212 x 160 to speed things up
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
        # and decide when to change direction and what direction to change to
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        # choosing direction
        self.x_err = x - (width/2)
        self.y_err = cy - (height/2)

        # TODO sanity check this
        if self.prev_direction[0] is not 'x' or self.prev_direction[0] is not 'y':
            if abs(x_err) > abs(y_err) and (self.prev_direction[0] is not 'x' or x_err / abs(x_err) is not self.prev_direction[1]):
                self.largest_err = self.x_err
                self.other_err = self.y_err
                self.err_axis = 'x'
                self.direction_msg = self.makeDirectionMessage(largest_err, err_axis)
            elif self.prev_direction[0] is not 'y' or y_err / abs(y_err) is not self.prev_direction[1]:
                self.largest_err = self.y_err
                self.other_err = self.x_err
                self.err_axis = 'y'
                self.direction_msg = self.makeDirectionMessage(largest_err, err_axis)
            else:
                self.direction_msg = self.stop
                self.prev_direction = ['', '']
        movement_pub.publish(self.keepCentered(self.other_err, self.err_axis))

def main():
    rospy.init_node('line_follower')
    line_follower_object = LineFollower()
    rospy.spin()




if __name__ == '__main__':
    main()
