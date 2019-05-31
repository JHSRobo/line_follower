#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        # real feed
        #self.image_sub = rospy.Subscriber("/rov/image", Image, self.camera_callback)
        # sim feed
        self.image_sub = rospy.Subscriber("/rov/camera1/image_raw", Image, self.camera_callback)
        self.pub = rospy.Publisher("/rov/cmd_vel", Twist, queue_size=0)

        # Main direction messages
        self.right = Twist()
        self.left = Twist()
        self.up = Twist()
        self.down = Twist()
        self.stop = Twist()

        # rough relationship between horizontal and vertical thrust
        # as well as forward and back
        self.baseThrust = 0.15
        self.reverseConstant = 1.5 # forward thrust is about 1.5x as powerful as reverse with the same cmd_vel
        self.upwardThrustRatio = math.sqrt(2) + (1/self.reverseConstant) * math.sqrt(2)
        self.downwardThrustRatio = math.sqrt(2) + self.reverseConstant * math.sqrt(2)

        # for picking which direction message to use
        # axes of an image increase left to right and bottom to top
        self.prev_contacts = []
        self.y_directions = {1 : self.down, -1 : self.up}
        self.x_directions = {1 : self.right, -1 : self.left}
        self.axes = {'x' : self.x_directions, 'y' : self.y_directions, 'stop' : self.stop}
        self.direction_msg = Twist()

        # for software PD controller (no integral)
        self.prev_err = 0
        self.prev_time = rospy.Time.now()
        self.kP = 0.003
        self.kD = 0.002
        self.longestLine = [0, 0, 0, 0]

        # for choosing directions
        # [axis ('x' or 'y'), direction (1 or -1)]
        # right = ['x', 1]
        # left = ['x', -1]
        # top = ['y', -1]
        # bottom = ['y', 1]
        self.curr_direction = [None, None]
        self.prev_direction = [None, None]
        self.contacts = []
        self.edge = [] #important edge for finding midpoint

        # cropping on image for finding boundary contacts
        self.borderWidth = 25

        # preprocessing parameters
        self.height = 212
        self.width = 212

        # for deciding when to end line following
        self.start_time = rospy.Time.now().to_sec()
        self.duration = 30 # in seconds
        self.isStopped = False

        # for ease of debug image showing
        self.cx = 0
        self.cy = 0
        self.contours = []

    def resetMessages(self):
        self.right.linear.x = self.baseThrust
        self.left.linear.x = -self.baseThrust
        self.up.linear.z = -self.baseThrust * self.upwardThrustRatio
        self.down.linear.z = self.baseThrust * self.downwardThrustRatio

    def resizeImage(self, ros_image):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        #end sim
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
        # convert to YCbCr for better color thresholding
        Y, Cr, Cb = cv2.split(cv2.cvtColor(self.img, cv2.COLOR_BGR2YCrCb))
        gamma_mask = cv2.adaptiveThreshold(Y, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,self.width-1,30)
        red_mask = cv2.adaptiveThreshold(Cr, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,self.width-1,-15)
        mask = cv2.bitwise_and(gamma_mask, red_mask)
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        self.mask = mask

    # returns an array of arrays of axis ('x' or 'y') and direction (1 or -1) of boundary contacts
    def findBoundaryContacts(self):
        self.contacts = []
        self.boundary = self.mask[self.borderWidth:-self.borderWidth, self.borderWidth:-self.borderWidth]
        if(np.count_nonzero(self.boundary, axis=0)[0] > 0): #left
            self.contacts.append(['x', -1])
        if(np.count_nonzero(self.boundary, axis=0)[-1] > 0): #right
            self.contacts.append(['x', 1])
        if(np.count_nonzero(self.boundary, axis=1)[0] > 0): #top
            self.contacts.append(['y', -1])
        if(np.count_nonzero(self.boundary, axis=1)[-1] > 0): #bottom
            self.contacts.append(['y', 1])

    # choose direction based off of spread, boundary contacts, and previous direction
    def chooseDirection(self):
        self.findBoundaryContacts()
        print 'contacts: ', self.contacts
        print 'prev conts: ', self.prev_contacts
        for contact in self.contacts:
            if contact not in self.prev_contacts:
                print("new conts")
                if contact[0] != self.prev_direction[0]:
                    print("change dir")
                    return contact
        return self.prev_direction

    def makeDirectionMessage(self, direction):
        try:
            return self.axes.get(direction[0], self.stop).get(direction[1], self.stop)
        except AttributeError as e:
            print(e)
            return self.stop

    def correctError(self, error):
        correction = 0
        pcorr = self.kP * error
        print 'perr: ', error
        #print "pcorr: ", pcorr
        try:
            print "pix change: ", (error - self.prev_err)
            #print "time diff: ", (rospy.Time.now() - self.prev_time).to_sec()
            dcorr = self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
        except ZeroDivisionError as e:
            print e
            dcorr = 0
        if np.isnan(dcorr):
            correction = pcorr
        correction = pcorr + dcorr
        self.prev_time = rospy.Time.now()
        self.prev_err = error
        #print "dcorr", dcorr
        print "correction", correction
        print 'time:', self.prev_time.to_sec()
        return correction

    def findLongestLine(self):
        lengths = []
        newsegs = []
        if self.segments is not None:
            for line in self.segments:
                if self.findLineDirection(line[0]) == self.curr_direction[0]:
                    lengths.append(self.calcLength(line[0]))
                    newsegs.append(line[0])
            index = lengths.index(max(lengths))
            self.longestLine = newsegs[index]
        else:
            print("no segments")
        print 'longest line: ', self.longestLine
        print 'length: ', self.calcLength(self.longestLine)
        return self.longestLine

    def findLineDirection(self, line):
        if abs(line[0] - line[2]) > abs(line[1] - line[3]):
            return 'x'
        elif abs(line[0] - line[2]) < abs(line[1] - line[3]):
            return 'y'
        else:
            return 'neither'

    def calcLength(self, line):
        # line[0] = x1
        # line[1] = y1
        # line[2] = x2
        # line[3] = y2
        return math.sqrt( ( line[0] - line[2] ) ** 2 + ( line[1] - line[3] ) ** 2 )

    def calcMidpoint(self):
        self.segments = cv2.HoughLinesP(self.mask, 5, math.pi/180, 50)
        self.findLongestLine()
        self.cx, self.cy = (self.longestLine[0] + self.longestLine[2])/2, (self.longestLine[1] + self.longestLine[3])/2
        print 'midpoint:', self.cx, self.cy

    def calcError(self):
        self.calcMidpoint()
        self.x_err = self.cx - (self.width/2)
        self.y_err = self.cy - (self.height/2)

    def keepCentered(self, axis):
        self.calcError()
        if axis is 'x': # travel along x axis --> correct y error
            if self.prev_direction[0] != 'x':
                print 'reset err bc change to x'
                self.prev_err = self.y_err
            correction = self.correctError(self.y_err)
            if np.sign(correction) > 0: #down
                self.direction_msg.linear.z = max(min(correction * self.downwardThrustRatio, 1), -1)
            else:
                self.direction_msg.linear.z = max(min(correction * self.upwardThrustRatio, 1), -1)
        elif axis is 'y': # travel along y axis --> correct x error
            if self.prev_direction[0] != 'y':
                print 'reset err bc change to y'
                self.prev_err = self.x_err
            self.direction_msg.linear.x = max(min(self.correctError(self.x_err), 1), -1)


    def showDebugImages(self, axis):
        cv2.imshow("resized", self.img)
        cv2.imshow("mask", self.mask)
        cv2.imshow("boundary", self.boundary)
        centroid = cv2.bitwise_and(self.img,self.img, mask= self.mask)
        lineColor = (255, 0, 0)
        cv2.line(centroid, (self.longestLine[0], self.longestLine[1]), (self.longestLine[2], self.longestLine[3]), lineColor, 1, 8)
        cv2.circle(centroid,(int(self.cx), int(self.cy)), 10,(0,255,0),-1)
        cv2.imshow("centroid", centroid)

    def checkIsStopped(self):
        if rospy.Time.now().to_sec() > (self.start_time + self.duration):
            if len(self.contacts) < 2:
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
            self.curr_direction = self.chooseDirection()
            print 'direction: ', self.curr_direction
            self.direction_msg = self.makeDirectionMessage(self.curr_direction)

            # error correction
            self.keepCentered(self.curr_direction[0])

            # debug image
            self.showDebugImages(self.curr_direction[0])
            cv2.waitKey(1)

            self.pub.publish(self.direction_msg)
            self.checkIsStopped()
            self.prev_contacts = self.contacts
            self.prev_direction = self.curr_direction # update direction
        else:
            print("Dam inspection complete")
            self.pub.publish(self.stop)
def main():
    rospy.init_node('line_follower')
    line_follower_object = LineFollower()
    rospy.spin()

if __name__ == '__main__':
    main()
