#!/usr/bin/env python

import cv_bridge
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist

bridge = cv_bridge.core.CvBridge()

global cv_image, new_message
cv_image = np.ndarray([640, 480])
new_message = False

# Load parameters for trackbars.
# If not loaded into parameter server fall back to defaults
lH = rospy.get_param("low_hue", 0)
lS = rospy.get_param("low_saturation", 72)
lV = rospy.get_param("low_value", 84)
hH = rospy.get_param("high_hue", 31)
hS = rospy.get_param("high_saturation", 140)
hV = rospy.get_param("high_value", 255)
threshold = rospy.get_param("~/threshold", 104)


MAX_ANG_VEL = 1.0 # Maximum angular velocity (Radians per second)

# Set parameters for blob detector
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 2000
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
blobparams.minArea = 100
blobparams.maxArea = 1000000000
blobparams.minDistBetweenBlobs = 400

# Initalize detector
detector1 = cv2.SimpleBlobDetector_create(blobparams)

def remap(value, firstMin, firstMax, secondMin, secondMax):
    leftSpan = firstMax - firstMin
    rightSpan = secondMax - secondMin
    valueScaled = float(value - firstMin) / float(leftSpan)
    return secondMin + (valueScaled * rightSpan)

def converter(msg):
    global new_message, cv_image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    new_message = True

def updateValuelH(new_value):
    global lH
    lH = new_value
    rospy.set_param("low_hue", lH)
    return
def updateValuelS(new_value):
    global lS
    lS = new_value
    rospy.set_param("low_saturation", lS)
    return
def updateValuelV(new_value):
    global lV
    lV = new_value
    rospy.set_param("low_value", lV)
    return
def updateValuehH(new_value):
    global hH
    hH = new_value
    rospy.set_param("high_hue", hH)
    return
def updateValuehS(new_value):
    global hS
    hS = new_value
    rospy.set_param("high_saturation", hS)
    return
def updateValuehV(new_value):
    global hV
    hV = new_value
    rospy.set_param("high_value", hV)
    return
def updateValue(new_value):
    global threshold
    threshold = new_value
    rospy.set_param("high_threshold", threshold)
    return

def onBtnChange(new_value):
    print(new_value)
    return

if __name__ == '__main__':
    global new_message, cv_image

    # Initialize ROS node, publishers and subscribers
    rospy.init_node('converter')
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("image_raw", Image, converter)

    # Set up trackbar callbacks
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
            
            keypoints1 = detector1.detect(thresh1)
            cv_image = cv2.drawKeypoints(cv_image, keypoints1, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            print("Keypoints: " + str(len(keypoints1)))
            mean_blob = 0
            for i in keypoints1:
                x = int(i.pt[0])
                y = int(i.pt[1])
                text = str(x)+';'+str(y)
                blob_size = i.size
                mean_blob += x
                cv2.putText(cv_image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            if len(keypoints1) != 0:
                mean_blob = mean_blob/len(keypoints1)
            cv2.imshow('Original', cv_image)            

            cmd_vel = Twist()
            cmd_vel.angular.z = remap(mean_blob, 0, cv_image.shape[1], -MAX_ANG_VEL, MAX_ANG_VEL)
            
            pub.publish(cmd_vel)

            # Display the resulting cv_image
            cv2.imshow('Processed', thresh1)
            cv2.waitKey(3)
            new_message = False

