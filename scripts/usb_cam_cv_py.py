#!/usr/bin/env python

import cv_bridge
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist

bridge = cv_bridge.core.CvBridge()

cv_image = np.ndarray([1280, 720])
new_message = False

lH = 0
lS = 72
lV = 84
hH = 31
hS = 140
hV = 255
threshold = 104

blobparams = cv2.SimpleBlobDetector_Params()

def remap(value, firstMin, firstMax, secondMin, secondMax):
            
    leftSpan = firstMax - firstMin
    rightSpan = secondMax - secondMin

    valueScaled = float(value - firstMin) / float(leftSpan)

    return secondMin + (valueScaled * rightSpan)

def converter(msg):
    global new_message, cv_image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    new_message = True


def init_image_converter():
    global blobparams
    rospy.init_node('converter')
    rospy.Subscriber("usb_cam/image_raw", Image, converter)
    blobparams.filterByCircularity = False
    blobparams.minDistBetweenBlobs = 2000
    blobparams.filterByInertia = False
    blobparams.filterByConvexity = False
    blobparams.minArea = 100
    blobparams.maxArea = 1000000000
    blobparams.minDistBetweenBlobs = 400

def updateValuelH(new_value):
    global lH
    lH = new_value
    return
def updateValuelS(new_value):
    global lS
    lS = new_value
    return
def updateValuelV(new_value):
    global lV
    lV = new_value
    return
def updateValuehH(new_value):
    global hH
    hH = new_value
    return
def updateValuehS(new_value):
    global hS
    hS = new_value
    return
def updateValuehV(new_value):
    global hV
    hV = new_value
    return
def updateValue(new_value):
    global threshold
    threshold = new_value
    return

if __name__ == '__main__':
    global new_message, cv_image, blobparams
    init_image_converter()
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    cv2.namedWindow('Original')
    cv2.createTrackbar("lower H", "Original", lH, 179, updateValuelH)
    cv2.createTrackbar("Upper H", "Original", hH, 179, updateValuehH)
    cv2.createTrackbar("lower S", "Original", lS, 255, updateValuelS)
    cv2.createTrackbar("Upper S", "Original", hS, 255, updateValuehS)
    cv2.createTrackbar("lower V", "Original", lV, 255, updateValuelV)
    cv2.createTrackbar("Upper V", "Original", hV, 255, updateValuehV)
    cv2.createTrackbar("Threshold", "Original", threshold, 255, updateValue)
    while not rospy.is_shutdown():
        if new_message:
            lowerLimits = np.array([lH, lS, lV])
            upperLimits = np.array([hH, hS, hV])

            thVesholded = cv2.inRange(cv_image, lowerLimits, upperLimits)
            outimage = cv2.bitwise_and(cv_image, cv_image, mask = thVesholded)
            gray = cv2.cvtColor(outimage, cv2.COLOR_BGR2GRAY)
            ret, thresh1 = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
            thresh1 = cv2.bitwise_not(thresh1)
            
            detector1 = cv2.SimpleBlobDetector_create(blobparams)
            keypoints1 = detector1.detect(thresh1)
            cv_image = cv2.drawKeypoints(cv_image, keypoints1, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            print(len(keypoints1))
            mean_blob = 0
            for i in keypoints1:
                x = int(i.pt[0])
                y = int(i.pt[1])
                text = str(x)+';'+str(y)
                blob_size = i.size
                mean_blob += x
                cv2.putText(cv_image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            if len(keypoints1) != 0:
                mean_blob = mean_blob/len(keypoints1) - 640
            cv2.imshow('Original', cv_image)            

            cmd_vel = Twist()
            cmd_vel.angular.z = remap(mean_blob, -640.0, 640.0, -1.0, 1.0)
            
            pub.publish(cmd_vel)

            # Display the resulting cv_image
            cv2.imshow('Processed', thresh1)
            cv2.waitKey(3)
            new_message = False

