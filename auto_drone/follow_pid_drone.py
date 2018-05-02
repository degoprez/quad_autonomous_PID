#!/usr/bin/env python

import numpy as np

import cv2 as cv
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,
                                           queue_size=1)
        self.twist = Twist()

        # PID controllers
        self.prev_err = 0
        self.prev_err_z = 0
        self.prev_integral = 0
        self.prev_integral_z = 0
        self.K_p = .004
        self.K_i = 0
        self.K_d = 0

        self.K_p_z = .02
        self.K_i_z = 0
        self.K_d_z = 0


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #lower_yellow = numpy.array([10, 10, 10])
        #upper_yellow = numpy.array([255, 255, 250])
        #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w = image.shape
        #search_top = 3*h/4
        #search_bot = 3*h/4 + 20
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        #M = cv2.moments(mask)
	ret, thresh = cv.threshold(image, 127, 255, 0)
	im2, contours,hierarchy = cv.findContours(thresh, 1, 2)

	try:
		cnt = contours[0]
	except IndexError:
		self.twist.linear.x = 0
		self.twist.angular.z = 0.6
		self.prev_err = 0
		self.prev_integral = 0
		self.prev_err_z = 0
		self.prev_integral_z = 0
		self.cmd_vel_pub.publish(self.twist)
		cv.imshow("window", image)
		cv.waitKey(3)
		return
	cnt = contours[0]
	M = cv.moments(cnt)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = float(cx - w/2)
            err_z = float(cy - h/2)
            integral = self.prev_integral + (err * 0.02)
            integral_z = self.prev_integral_z + (err_z * 0.02)
            derivative = (err - self.prev_err) / 0.02
            derivative_z = (err_z - self.prev_err_z) / 0.02
            output = -1 * ((self.K_p*err) + (self.K_i*integral) + (self.K_d*derivative) + 0)
            output_z = -1 * ((self.K_p_z*err_z) + (self.K_i_z*integral_z) + (self.K_d_z*derivative_z) + 1)
            self.twist.linear.x = 1.0
            self.twist.linear.z = output_z
            self.twist.angular.z = output
            self.cmd_vel_pub.publish(self.twist)
            self.prev_err = err
            self.prev_integral = integral
            self.prev_err_z = err_z
            self.prev_integral_z = integral_z
        cv.imshow("window", image)
        cv.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
