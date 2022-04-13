#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np
import cv2

rospy.init_node('line_tracker')
line_coord_pub = rospy.Publisher("line_coordinates", Float64, queue_size=1)
debug_image_pub = rospy.Publisher("line_tracker_img", Image, queue_size=10)
bridge = CvBridge()

def getContourWithMaxAR(contours):
    max_ar = 0
    max_contour = contours[0]
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w) / h
        if aspect_ratio > max_ar:
            max_contour = c
            max_ar = aspect_ratio

    return max_contour

def image_cb(msg):
    frame = bridge.imgmsg_to_cv2(msg)
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Color thresholding
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    # Find the contours of the frame
    contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # Find the biggest contour (if detected)

    if len(contours) > 0:
        c = getContourWithMaxAR(contours)
        M = cv2.moments(c)
        cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0
        cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0
        cv2.line(blur, (cx, 0), (cx, msg.height), (255, 0, 0), 1)
        cv2.line(blur, (0, cy), (msg.width, cy), (255, 0, 0), 1)
        cv2.drawContours(blur, contours, -1, (0, 255, 0), 1)
        line_coord_pub.publish((cy/msg.height)*2-1)
        debug_image_pub.publish(bridge.cv2_to_imgmsg(blur))

    # else:
    #     print("No lines detected")

    # cv2.imshow('frame', blur)

rospy.Subscriber("/INSERT/IMAGE/TOPIC", Image, image_cb)
rospy.spin()