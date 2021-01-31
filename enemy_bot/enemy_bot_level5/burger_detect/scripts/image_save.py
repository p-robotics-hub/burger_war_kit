#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
#

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import sys
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import os

dir="Image/test/"
num=10*1000
class ImageGet():
    def __init__(self):

        rospy.Subscriber('/image_raw', Image, self.Image_save)
        self.bridge = CvBridge()
        self.count=0
    def Image_save(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("sample.jpg",cv_image)
        cv2.waitKey(5)
        #cv2.imwrite(dir+"sample"+repr(self.count)+".jpg",cv_image)
        print("save done.")
        #self.count+=1

    def get_image(self):
        r = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():

            r.sleep()
            if self.count>num:
                break

if __name__ == '__main__':
    if not os.path.exists(dir):
        os.mkdir(dir)
    rospy.init_node('get_image')
    bot = ImageGet()
    bot.get_image()
