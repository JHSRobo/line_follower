#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        # figure out the video feed
        # sim feed
        self.image_sub = rospy.Subscriber("/rov/camera1/image_raw", Image, self.camera_callback)
        # real feed
        #self.image_sub = rospy.Subscriber("/rov/image_raw", Image, self.camera_callback)
        self.movement_pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size=0)
        # dictionary of 4 cardinal directions and one stop for line follow task
        self.right = Twist()
        self.left = Twist()
        self.up = Twist()
        self.down = Twist()
        self.stop = Twist()

        self.right.linear.x = 0.088
        self.left.linear.x = -0.088
        self.up.linear.z = -0.125
        self.down.linear.z = 0.125

        self.corrected_msg = Twist()
        # for deciding orientation of line3
        # rows are first index, columns are second
        self.largestRowValue = 0
        self.largestColValue = 0
        # axes of an image increase left to right and bottom to top
        self.y_directions = {1 : self.down, -1 : self.up}
        self.x_directions = {1 : self.right, -1 : self.left}
        self.axes = {'x' : self.x_directions, 'y' : self.y_directions, 'stop' : self.stop}
        self.direction_msg = Twist()
        # first value is the axis and the second is the direction
        self.prev_direction =['', '']

        self.y_err = 0
        self.x_err = 0
        self.err = 0
        self.err_axis = ''
        # for PD (no integral)
        self.prev_err = 0
        self.prev_time = rospy.Time.now()
        self.kP = 0.00225
        self.kD = 0.0007

    def makeDirectionMessage(self, error, axis):
        try:
            self.prev_direction = [axis, error / abs(error)]
            print("y: ", self.y_err)
            print("x: ", self.x_err)
            print("dir", axis, error/abs(error))
            print("prev dir", self.prev_direction)
            return self.axes.get(axis).get(error / abs(error))
        except ZeroDivisionError:
            return self.axes.get(self.prev_direction[0], self.axes).get(self.prev_direction[1], self.stop)

    # funky PD controller
    # (abs(error)/error)*(abs(error) ** 0.44)/35
    def correctError(self, error):
        correction = 0
        try:
            print("dcorr: ", self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec())
            print("derr: ", (error - self.prev_err))
            correction = self.kP * error + self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
        except ZeroDivisionError:
            print("dcorr: NaN no time")
            correction = self.kP * error
        self.prev_time = rospy.Time.now()
        self.prev_err = error
        print("pcorr: ", self.kP * error)
        print("correction", correction)
        print('time:', self.prev_time)
        return correction

    def calcSpread(self, img):
        height, width = img.shape
        colSums = [0] * width
        rowSums = []
        for row in img:
            rowSums.append(sum(row))
        for column in range(height):
            for row in img:
                colSums[column] += row[column]
        self.largestRowValue = max(rowSums)
        self.largestColValue = max(colSums)

    def keepCentered(self, error, axis):
        print("correcting error on", axis)
        print("error", error)
        self.corrected_msg = self.direction_msg
        if axis is 'x':
            self.corrected_msg.linear.x = self.correctError(error)
        elif axis is 'y':
            self.corrected_msg.linear.z = self.correctError(error)
        return self.corrected_msg

    def camera_callback(self, data):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # downsampling to 212 x 160 to speed things up
        img_height, img_width, channels = image.shape
        height = 160
        width = 160
        newsize = (width, height)
        interpolation = cv2.INTER_NEAREST
        # cropping image to one half because sim cam has something blocking it
        cropped_img = image[int(math.floor((img_height - img_height/1.5)/2)):int(math.ceil((img_height + img_height/1.5)/2)), int(math.floor((img_width - img_width/1.5)/2)):int(math.ceil((img_width + img_width/1.5)/2))]
        resized_img = cv2.resize(cropped_img, newsize, 0, 0, interpolation)

        # convert to hsv for better color thresholding
        hsv = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)

        # masking image to red and not red since only those colors matter
        # red goes around 360 (about 330 to 30) but cv sucks so its hues (and vals and saturations)go 0 to 255
        # so we'll just say red starts at 160 and ends at 20
        # https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
        lower_red = np.array([160, 60, 60])
        inbw_red1 = np.array([255, 255, 255])
        inbw_red2 = np.array([0, 60, 60])
        upper_red = np.array([20, 255, 255])
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

        #for debug
        #cv2.imshow("original", image)
        #cv2.imshow("resized hsv", hsv)
        #cv2.imshow("mask", mask)
        centroid = cv2.bitwise_and(resized_img,resized_img, mask= mask)
        cv2.circle(centroid,(int(cx), int(cy)), 10,(0,255,0),-1)
        cv2.imshow("centroid", centroid)
        cv2.waitKey(1)
        self.x_err = cx - (width/2)
        self.y_err = cy - (height/2)

        # update axis of direction
        self.calcSpread(mask)

        # TODO sanity check this direction choosing
        if self.prev_direction[0] is not 'x' and self.prev_direction[0] is not 'y':
            if self.largestColValue > self.largestRowValue:
                # gotta start with least amount of line in frame
                print('y choose')
                self.direction_msg = self.makeDirectionMessage(self.y_err, 'y')
                # error to correct
                self.err = self.x_err
                self.err_axis = 'x'
            else:
                # gotta start with least amount of line in frame
                print('x choose')
                self.direction_msg = self.makeDirectionMessage(self.x_err, 'x')
                # error to correct
                self.err = self.y_err
                self.err_axis = 'y'
        elif self.prev_direction[0] is 'x':
            # error to correct
            self.err = self.y_err
            self.err_axis = 'y'
            # changing direction
            if self.largestColValue > self.largestRowValue:
                # gotta start with least amount of line in frame
                print('y choose in x')
                self.direction_msg = self.makeDirectionMessage(self.y_err, 'y')
                # error to correct
                self.err = self.x_err
                self.err_axis = 'x'
        elif self.prev_direction[0] is 'y':
            # error to correct
            self.err = self.x_err
            self.err_axis = 'x'
            # changing direction
            if self.largestRowValue > self.largestColValue:
                # gotta start with least amount of line in frame
                print('x choose in y')
                self.direction_msg = self.makeDirectionMessage(self.x_err, 'x')
                # error to correct
                self.err = self.y_err
                self.err_axis = 'y'
        # error correction
        self.movement_pub.publish(self.keepCentered(self.err, self.err_axis))

def main():
    rospy.init_node('line_follower')
    line_follower_object = LineFollower()
    rospy.spin()




if __name__ == '__main__':
    main()
