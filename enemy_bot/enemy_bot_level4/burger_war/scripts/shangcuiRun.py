#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from enum import Enum
import cv2
import numpy as np
import json

class ActMode(Enum):
    SEARCH = 1
    SNIPE  = 2
    ESCAPE = 3
    MOVE   = 4

class SeigoBot():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # Lidar
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('/red_bot/scan', LaserScan, self.lidarCallback)
        
        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/red_bot/image_raw', Image, self.imageCallback)

	# war status
	self.war_state = rospy.Subscriber("/red_bot/war_state", String, self.stateCallback)

        self.my_score = 0
        self.act_mode = ActMode.SEARCH
    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        # print(self.scan)

    def find_rect_of_blue_marker(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # blue detection
        lower_blue = np.array([130, 50, 50])
        upper_blue = np.array([200, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects

    def find_rect_of_enemy(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # red detection
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects
        
    def find_rect_of_green_marker(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # green detection
        lower_green = np.array([75, 50, 50])
        upper_green = np.array([110, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects
        
    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # print(self.img);
        frame = self.img
        rects = self.find_rect_of_blue_marker(frame)
        if len(rects) > 0:
            self.act_mode = ActMode.SNIPE
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            
        #    if self.camera_preview:
        # print("image show")
        cv2.imshow("Image window", frame)
        cv2.waitKey(1)

    def stateCallback(self, state):
        print(state.data)
        dic = json.loads(state.data)
        tmp = int(dic["scores"]["r"])
	if tmp > self.my_score and self.act_mode == ActMode.SNIPE:
            self.act_mode = ActMode.SEARCH
        self.my_score = tmp
        print(self.my_score)

    def approachToMarker(self):
        x = 0
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # Main Loop --->
        while not rospy.is_shutdown():
            print(self.act_mode)
            if self.act_mode == ActMode.SEARCH:
                twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()

        # Main Loop <---
        
if __name__ == '__main__':
    rospy.init_node('seigo_run')
    bot = SeigoBot('Seigo')
    bot.strategy()

