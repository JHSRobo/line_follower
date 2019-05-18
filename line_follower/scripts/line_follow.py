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

        # rough relationship between horizontal and vertical thrust
        # as well as forward and back
        self.baseThrust = 0.12
        self.reverseConstant = 1.5 # forward thrust is about 1.5x as powerful as reverse with the same cmd_vel
        self.upwardThrustRatio = math.sqrt(2) + (1/self.reverseConstant) * math.sqrt(2)
        self.downwardThrustRatio = math.sqrt(2) + self.reverseConstant * math.sqrt(2)

        # for picking which direction message to use
        # axes of an image increase left to right and bottom to top
        self.y_directions = {1 : self.down, -1 : self.up}
        self.x_directions = {1 : self.right, -1 : self.left}
        self.axes = {'x' : self.x_directions, 'y' : self.y_directions, 'stop' : self.stop}
        self.direction_msg = Twist()

        # for software PD controller (no integral)
        self.prev_err = 0
        self.prev_time = rospy.Time.now()
        self.kP = 0.003
        self.kD = 0.002

        # for choosing directions
        # [axis ('x' or 'y'), direction (1 or -1)]
        # right = ['x', 1]
        # left = ['x', -1]
        # top = ['y', -1]
        # bottom = ['y', 1]
        self.curr_direction = [None, None]
        self.prev_direction = [None, None]
        self.largestColValue = 0
        self.largestRowValue = 0
        self.contacts = []
        self.edge = [] #important edge for finding midpoint

        # cropping on image for finding boundary contacts
        self.borderWidth = 25

        # preprocessing parameters
        self.height = 212
        self.width = 212

        # for deciding when to end line following
        self.start_time = rospy.Time.now().to_sec()
        self.duration = 90 # in seconds
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
        # red goes around 360 (about 330 to 30) but cv sucks so its hues go 0 to 180
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
        self.contacts = []
        mask = self.mask[self.borderWidth:-self.borderWidth, self.borderWidth:-self.borderWidth]
        if(np.count_nonzero(mask, axis=0)[0] > 0): #left
            self.contacts.append(['x', -1])
        if(np.count_nonzero(mask, axis=0)[-1] > 0): #right
            self.contacts.append(['x', 1])
        if(np.count_nonzero(mask, axis=1)[0] > 0): #top
            self.contacts.append(['y', -1])
        if(np.count_nonzero(mask, axis=1)[-1] > 0): #bottom
            self.contacts.append(['y', 1])
        print 'contacts: ', self.contacts

    # calculates whether a row or a column has more mask in it
    def calcSpread(self):
        print 'calcspread'
        height, width = self.mask.shape
        print np.amax(np.count_nonzero(self.mask, axis=0)) #col
        print np.amax(np.count_nonzero(self.mask, axis=1)) #row
        self.largestRowValue = np.amax(np.count_nonzero(self.mask, axis=1))
        self.largestColValue = np.amax(np.count_nonzero(self.mask, axis=0)) * width / height # scale factor in case of nonsquare image
        if self.largestColValue > self.largestRowValue:
            return 'y'
        else:
            return 'x'

    # choose direction based off of spread, boundary contacts, and previous direction
    def chooseDirection(self):
        axis = self.calcSpread()
        direction = self.prev_direction[1]
        self.findBoundaryContacts()
        if axis == self.prev_direction[0]: # no reversing
            print 'no reverse'
            return self.prev_direction
        for contact in self.contacts:
            print 'contact: ', contact
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
            print "time diff: ", (rospy.Time.now() - self.prev_time).to_sec()
            dcorr = self.kD * (error - self.prev_err) / (rospy.Time.now() - self.prev_time).to_sec()
        except ZeroDivisionError as e:
            print e
            dcorr = 0
        if np.isnan(dcorr):
            correction = pcorr
        correction = pcorr + dcorr
        self.prev_time = rospy.Time.now()
        self.prev_err = error
        print "dcorr", dcorr
        print "correction", correction
        print 'time:', self.prev_time.to_sec()
        return correction

    def calcMidpoint(self):
        _, contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        self.contours = np.array(contours) # shape should be (1, numberOfContours, 1, 2)
        direction = [None, None]
        direction[0], direction[1] = self.curr_direction[0], self.curr_direction[1]
        self.cy, self.cx = self.mask.shape[0]/2, self.mask.shape[1]/2
        print 'contacts: ', self.curr_direction in self.contacts
        if not self.curr_direction in self.contacts:
            print 'check other dir'
            direction[1] = -direction[1]
        if direction[0] == 'x':
            if direction[1] == -1: #left
                self.edge = np.where(self.contours[:,:,:,0] == self.borderWidth)
            else: #right
                self.edge = np.where(self.contours[:,:,:,0] == self.mask.shape[1]-self.borderWidth)
            if np.any(self.edge):
                self.cy = abs(self.contours[self.edge][0][1]-self.contours[self.edge][-1][1])/2 + np.amin(self.contours[self.edge][:,1])
                self.cx = self.mask.shape[0]/2
        if direction[0] == 'y':
            if direction[1] == -1: #top
                self.edge = np.where(self.contours[:,:,:,1] == self.borderWidth)
            else: #bottom
                self.edge = np.where(self.contours[:,:,:,1] == self.mask.shape[0]-self.borderWidth)
            if np.any(self.edge):
                self.cx = abs(self.contours[self.edge][0][0]-self.contours[self.edge][-1][0])/2 + np.amin(self.contours[self.edge][:,0])
                self.cy = self.mask.shape[1]/2
        print 'midpoint: ', self.cx, self.cy

    def calcError(self, axis):
        self.calcMidpoint()
        self.x_err = self.cx - (self.width/2)
        self.y_err = self.cy - (self.height/2)

    def keepCentered(self, axis):
        self.calcError(axis)
        if axis is 'x': # travel along x axis --> correct y error
            if self.prev_direction[0] != 'x':
                print 'reset err bc change to x'
                self.prev_err = self.y_err
            correction = self.correctError(self.y_err)
            if np.sign(correction) > 0: #down
                self.direction_msg.linear.z = correction * self.downwardThrustRatio
            else:
                self.direction_msg.linear.z = correction * self.upwardThrustRatio
        elif axis is 'y': # travel along y axis --> correct x error
            if self.prev_direction[0] != 'y':
                print 'reset err bc change to y'
                self.prev_err = self.x_err
            self.direction_msg.linear.x = self.correctError(self.x_err)


    def showDebugImages(self, axis):
        cv2.imshow("resized", self.img)
        cv2.imshow("mask", self.mask)
        centroid = cv2.bitwise_and(self.img,self.img, mask= self.mask)
        for point in self.contours[self.edge]:
            print 'point: ', tuple(point)
            cv2.circle(centroid, tuple(point), 3, (255,0,0), -1)
        cv2.circle(centroid,(int(self.cx), int(self.cy)), 10,(0,255,0),-1)
        cv2.imshow("centroid", centroid)

    def checkIsStopped(self):
        if rospy.Time.now().to_sec() > (self.start_time + self.duration):
            numContacts = 0
            for contact in self.contacts:
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
            self.curr_direction = self.chooseDirection()
            print 'direction: ', self.curr_direction
            print 'longest row: ', self.largestRowValue
            print 'longest col: ', self.largestColValue
            self.direction_msg = self.makeDirectionMessage(self.curr_direction)

            # error correction
            self.keepCentered(self.curr_direction[0])

            # debug image
            self.showDebugImages(self.curr_direction[0])
            cv2.waitKey(1)

            self.pub.publish(self.direction_msg)
            self.checkIsStopped()
            self.prev_direction = self.curr_direction # update direction
        else:
            self.pub.publish(self.stop)
def main():
    rospy.init_node('line_follower')
    line_follower_object = LineFollower()
    rospy.spin()

if __name__ == '__main__':
    main()
