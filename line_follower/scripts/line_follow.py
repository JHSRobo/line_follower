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
        # real feed
        #self.image_sub = rospy.Subscriber("/rov/image_raw", Image, self.camera_callback)
        # sim feed
        self.image_sub = rospy.Subscriber("/rov/camera1/image_raw", Image, self.camera_callback)
        self.pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size=0)

        # Main direction messages
        self.right = Twist()
        self.left = Twist()
        self.up = Twist()
        self.down = Twist()
        self.stop = Twist()

        # for picking which direction message to use
        # axes of an image increase left to right and bottom to top
        self.y_directions = {1 : self.down, -1 : self.up}
        self.x_directions = {1 : self.right, -1 : self.left}
        self.axes = {'x' : self.x_directions, 'y' : self.y_directions, 'stop' : self.stop}
        self.direction_msg = Twist()

        # for software PD controller (no integral)
        self.prev_err = 0
        self.prev_time = rospy.Time.now()
        self.kP = 0.002
        self.kD = 0

        # for choosing directions
        # [axis ('x' or 'y'), direction (1 or -1)]
        # right = ['x', 1]
        # left = ['x', -1]
        # top = ['y', -1]
        # bottom = ['y', 1]
        self.prev_direction = [None, None]
        self.largestColValue = 0
        self.largestRowValue = 0
        # cropping on image for finding boundary contacts
        self.borderWidth = 30

        # preprocessing parameters
        self.height = 160
        self.width = 160

        # for deciding when to end line following
        self.start_time = rospy.Time.now().to_sec()
        self.duration = 90 # in seconds
        self.isStopped = False

        # for ease of debug image showing
        self.segments = []
        self.cx = 0
        self.cy = 0
        self.longestLine = [0,0,0,0]

    def resetMessages(self):
        self.right.linear.x = 0.12
        self.left.linear.x = -0.12
        self.up.linear.z = -0.12
        self.down.linear.z = 0.12

    def resizeImage(self, ros_image):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        # downsampling to 212 x 160 to speed things up
        img_height, img_width, channels = image.shape
        newsize = (self.width, self.height)
        interpolation = cv2.INTER_NEAREST
        # no crop
        #cropped_img = image
        # square crop
        cropped_img = image[0:img_height, (img_width - img_height)/2:(img_width + img_height)/2]
        # cropping image to one half because sim cam has something blocking it
        #cropped_img = image[int(math.floor((img_height - img_height/1.5)/2)):int(math.ceil((img_height + img_height/1.5)/2)), int(math.floor((img_width - img_width/1.5)/2)):int(math.ceil((img_width + img_width/1.5)/2))]
        self.img = cv2.resize(cropped_img, newsize, 0, 0, interpolation)


    def maskImage(self):
        # convert to hsv for better color thresholding
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
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
        self.mask = mask

    # returns an array of arrays of axis ('x' or 'y') and direction (1 or -1) of boundary contacts
    def findBoundaryContacts(self):
        height, width = self.mask.shape
        # "crop" in to avoid occluded camera views
        frame = self.borderWidth/2
        height -= frame
        width -= frame
        contacts = []
        right = False
        left = False
        for row in self.mask:
            if row[width] > 0: # right side
                right = True
            if row[frame] > 0: # left side
                left = True
        if right:
            contacts.append(['x', 1])
        if left:
            contacts.append(['x', -1])
        if max(self.mask[height]) > 0: # bottom
            contacts.append(['y', 1])
        if max(self.mask[frame]) > 0: # top
            contacts.append(['y', -1])
        print 'contacts: ', contacts
        return contacts

    # calculates whether a row or a column has more mask in it
    def calcSpread(self):
        height, width = self.mask.shape
        colSums = [0] * width
        rowSums = []
        for row in self.mask:
            rowSums.append(sum(row))
        for column in range(width):
            for row in self.mask:
                colSums[column] += row[column] * width / height # scale factor in case of nonsquare image
        self.largestRowValue = max(rowSums)
        self.largestColValue = max(colSums)
        if self.largestColValue > self.largestRowValue:
            return 'y'
        else:
            return 'x'

    # choose direction based off of spread, boundary contacts, and previous direction
    def chooseDirection(self):
        axis = self.calcSpread()
        direction = self.prev_direction[1]
        if axis == self.prev_direction[0]: # no reversing
            return self.prev_direction
        contacts = self.findBoundaryContacts()
        for contact in contacts:
            if contact[0] != self.prev_direction[0]:
                direction = contact[1]
        return [axis, direction]

    def makeDirectionMessage(self, direction):
        return self.axes.get(direction[0], self.stop).get(direction[1], self.stop)

    def correctError(self, error):
        correction = 0
        pcorr = self.kP * error
        print 'perr: ', error
        print "pcorr: ", pcorr
        try:
            print "pix change: ", (error - self.prev_err)
            print "derr: ", (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
            print "dcorr: ", self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
            correction = pcorr + self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
        except ZeroDivisionError:
            print("dcorr: NaN no time")
            correction = pcorr
        self.prev_time = rospy.Time.now()
        self.prev_err = error
        print "correction", correction
        print 'time:', self.prev_time.to_sec()
        return correction

    def findLineDirection(self, line):
        if abs(line[0] - line[2]) > abs(line[1] - line[3]):
            # print 'line dir: x'
            return 'x'
        else:
            # print 'line dir: y'
            return 'y'

    def calcLength(self, line):
        # line[0] = x1
        # line[1] = y1
        # line[2] = x2
        # line[3] = y2
        return math.sqrt( ( line[0] - line[2] ) ** 2 + ( line[1] - line[3] ) ** 2 )

    def calcMidpoint(self, line):
        return (line[0] + line [2])/2, (line[1] + line[3])/2

    def findLongestLine(self, axis):
        lengths = []
        for line in self.segments:
            if self.findLineDirection(line[0]) == axis:
                lengths.append(self.calcLength(line[0]))
        index = lengths.index(max(lengths))
        self.longestLine = self.segments[index][0]
        print 'longest line: ', self.segments[index][0]
        print 'length: ', max(lengths)
        return self.segments[index][0]

    def calcError(self, axis):
        try:
            self.cx, self.cy = self.calcMidpoint(self.findLongestLine(axis))
        except TypeError as e:
            print e
            self.cx, self.cy = self.width/2, self.height/2
        print 'cxcy', self.cx, self.cy
        self.x_err = self.cx - (self.width/2)
        self.y_err = self.cy - (self.height/2)

    def keepCentered(self, axis):
        self.segments = cv2.HoughLinesP(self.mask, 5, math.pi/180, 50)
        self.calcError(axis)
        if axis is 'x': # travel along x axis --> correct y error
            if self.prev_direction[0] != 'x':
                print 'reset err bc change to x'
                self.prev_err = self.y_err
            self.direction_msg.linear.z = self.correctError(self.y_err)
        elif axis is 'y': # travel along y axis --> correct x error
            if self.prev_direction[0] != 'y':
                print 'reset err bc change to y'
                self.prev_err = self.x_err
            self.direction_msg.linear.x = self.correctError(self.x_err)


    def showDebugImages(self, axis):
        cv2.imshow("resized", self.img)
        cv2.imshow("mask", self.mask)
        centroid = cv2.bitwise_and(self.img,self.img, mask= self.mask)
        lineColor = (255, 0, 0)
        line = self.longestLine
        cv2.line(centroid, (line[0], line[1]), (line[2], line[3]), lineColor, 1, 8)
        cv2.circle(centroid,(int(self.cx), int(self.cy)), 10,(0,255,0),-1)
        cv2.imshow("centroid", centroid)

    def checkIsStopped(self):
        if rospy.Time.now().to_sec() > (self.start_time + self.duration):
            contacts = self.findBoundaryContacts()
            numContacts = 0
            for contact in contacts:
                numContacts += 1
            if numContacts < 2:
                self.isStopped = True
            else:
                self.isStopped = False

    def camera_callback(self, data):
        print '-----======== start callback ========-----'
        self.resetMessages()
        if not self.isStopped:
            # preprocessing
            self.resizeImage(data)
            self.maskImage()

            # cardinal direction
            direction = self.chooseDirection()
            print 'direction: ', direction
            print 'longest row: ', self.largestRowValue
            print 'longest col: ', self.largestColValue
            self.direction_msg = self.makeDirectionMessage(direction)

            # error correction
            self.keepCentered(direction[0])

            # debug image
            self.showDebugImages(direction[0])
            cv2.waitKey(1)

            self.pub.publish(self.direction_msg)
            self.checkIsStopped()
            self.prev_direction = direction # update direction
        else:
            self.pub.publish(self.stop)
def main():
    rospy.init_node('line_follower')
    line_follower_object = LineFollower()
    rospy.spin()

if __name__ == '__main__':
    main()
