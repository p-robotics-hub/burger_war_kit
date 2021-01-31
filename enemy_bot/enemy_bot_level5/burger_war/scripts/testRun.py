#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import json

import numpy as np
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib_msgs

# camera image 640*480
img_w = 640
img_h = 480
image_resize_scale = 1 # 8

# PI
PI = 3.1415
DEGRAD = 3.141592/180

# robot running coordinate in BASIC MODE
#basic_coordinate = np.array([
#    # x,    y,    th(deg)
#    [-1.0 , 0.3 ,  30],  # 1
#    [-1.0 ,-0.3 , 330],  # 2
#    [-0.6 , 0.0 ,   0],  # 3
#    [-0.5 ,-0.1 , 315],  # 4
#    [ 0   ,-0.6 , 180],  # 5
#    [ 0   ,-0.6 ,  90],  # 6
#    [ 0   ,-0.5 ,   0],  # 7
#    [ 0.5 ,-0.1 ,  45],  # 10
#
#    [ 1.0 ,-0.3 , 210],  # 1
#    [ 1.0 , 0.3 , 150],  # 2
#    [ 0.6 , 0.0 , 180],  # 3
#    [ 0.5 , 0.1 , 135],  # 4
#    [ 0   , 0.6 ,   0],  # 5
#    [ 0   , 0.6 , 270],  # 6
#    [ 0   , 0.5 , 180],  # 7
#    [-0.5 , 0.1 , 225]]  # 10
#)

target_coordinate = np.array([
#   [[ 1.20, 0.0 , 180],
   [[ 1.00, 0.3 , 150],
    [ 0.55, 0.0 , 180],
    [ 1.00,-0.3 , 210],
    [ 0.9 ,-0.4 , 235]],
#   [[-0.1 , 0.7 , 300],
   [[ 0   , 0.6 ,   0],
    [ 0   , 0.6 , 270],
    [ 0   , 0.6 , 180],
    [ 0.4 , 0.9 , 325]],
#   [[-1.2, -0.0 ,   0],
   [[-1.00,-0.3 , 330],
    [-0.55, 0.0 ,   0],
    [-1.00, 0.3 ,  30],
    [-0.9 , 0.4 ,  55]],
#   [[ 0.1 ,-0.7 , 120],
   [[ 0   ,-0.6 , 180],
    [ 0   ,-0.6 ,  90],
    [ 0   ,-0.6 ,   0],
    [-0.4 ,-0.9 , 145]]
])

#    [-0.4, 0.0, 0],  # 1
#    [-0.9, 0.0, 0],  # 2
#    [-0.9, 0.4, 0],  # 3
#    [-0.9, -0.4, 0], # 4
#    [-0.9, 0.0, 0],  # 5
#    [0, -0.5, 0],    # 6
#    [0, -0.5, PI],   # 7
#    [0, -0.5, PI/2], # 8
#    [0, -1.2, PI/2]] # 17

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # odom
        topicname_odom = "odom"
        self.odom = rospy.Subscriber(topicname_odom, Odometry, self.odomCallback)

        # amcl pose
        topicname_amcl_pose = "amcl_pose"
        self.amcl_pose = rospy.Subscriber(topicname_amcl_pose, PoseWithCovarianceStamped, self.AmclPoseCallback)

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)

        self.basic_mode_process_step_idx = 0 # process step in basic MODE

        self.scan_ave = np.zeros((2,12))        # [0]:latest, [1]:prev
        self.scan_diff = np.zeros(12)
        self.scan_sum = np.zeros(16)
        self.myPosX = 0
        self.myPosY = -150
        self.myDirect = np.pi / 2

        ## war status
        #topicname_war_state = "war_state"
	#self.war_state = rospy.Subscriber(topicname_war_state, String, self.stateCallback)
        #self.my_score = 0
        #self.enemy_score = 0

    def odomCallback(self, data):
        # print(data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        e = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        # print(e[2] / (2 * np.pi) * 360)
        self.myDirect = e # rad

    def AmclPoseCallback(self, data):
        self.myPosX = data.pose.pose.position.x
        self.myPosY = data.pose.pose.position.y
        # print(self.myPosX, self.myPosY)

    # camera image call back sample
    # convert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        size = (img_w/image_resize_scale, img_h/image_resize_scale)
        frame = cv2.resize(self.img, size)

        if self.camera_preview:
            #print("image show")
            cv2.imshow("Image window", frame)
            cv2.waitKey(1)

    def stateCallback(self, state):
        # print(state.data)
        dic = json.loads(state.data)

        if self.name == "red_bot": # red_bot
            self.my_score = int(dic["scores"]["r"])
            self.enemy_score = int(dic["scores"]["b"])
        else: # blue_bot
            self.my_score = int(dic["scores"]["b"])
            self.enemy_score = int(dic["scores"]["r"])

        print "Zone0", dic["targets"][ 8]["player"],dic["targets"][14]["player"],dic["targets"][ 6]["player"] 
	print "Zone1", dic["targets"][ 7]["player"],dic["targets"][16]["player"],dic["targets"][10]["player"] 
	print "Zone2", dic["targets"][11]["player"],dic["targets"][17]["player"],dic["targets"][13]["player"] 
	print "Zone3", dic["targets"][12]["player"],dic["targets"][15]["player"],dic["targets"][ 9]["player"] 
    # Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    # Ref: https://github.com/hotic06/burger_war/blob/master/burger_war/scripts/navirun.py
    # RESPECT @hotic06
    # do following command first.
    #   $ roslaunch burger_navigation multi_robot_navigation_run.launch
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return -1

        get_state = self.client.get_state()
        print("wait", wait, "get_state", get_state)
        if get_state == 2:  # if send_goal is canceled
            return -1

        return 0

    def cancelGoal(self):
        self.client.cancel_goal()

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

        self.scan_ave[1] = self.scan_ave[0]        # prev <= latest
        self.scan_ave[0,0] = (sum(self.scan.ranges[0:2])+sum(self.scan.ranges[358:359])) * 200 # /5 * 1000
        if self.scan_ave[0,0] == float('inf'):
            self.scan_ave[0,0] = 100
        i = 1
        while i < 12:
            self.scan_ave[0,i] = sum(self.scan.ranges[i*30-2:i*30+2]) * 200 # /5 * 1000
            if self.scan_ave[0,i] == float('inf'):
                self.scan_ave[0,i] = 100
            i += 1
        self.scan_diff = self.scan_ave[0] - self.scan_ave[1]

        # RESPECT @koy_tak        
#        if (self.scan.ranges[0] != 0 and self.scan.ranges[0] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[10] != 0 and self.scan.ranges[10] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[350] != 0 and self.scan.ranges[350] < DISTANCE_TO_WALL_THRESHOLD):
#            self.f_isFrontBumperHit = True
#            print("self.f_isFrontBumperHit = True")
#            self.cancelGoal()
#        else:
#            self.f_isFrontBumperHit = False

    def calcTwist(self, direction):
        if direction == 0:
            fr  = self.scan_ave[0,0]
            f30 = self.scan_ave[0,1]
            f60 = self.scan_ave[0,2]
            side= self.scan_ave[0,3]
            b60 = self.scan_ave[0,4]
            b30 = self.scan_ave[0,5]
            bo  = self.scan_ave[0,7]
            sign_x = 1
            sign_rot = 1
        elif direction == 1:
            fr  = self.scan_ave[0,6]
            f30 = self.scan_ave[0,5]
            f60 = self.scan_ave[0,4]
            side= self.scan_ave[0,3]
            b60 = self.scan_ave[0,2]
            b30 = self.scan_ave[0,1]
            bo  = self.scan_ave[0,11]
            sign_x = -1
            sign_rot = -1
        elif direction == 2:
            fr  = self.scan_ave[0,0]
            f30 = self.scan_ave[0,11]
            f60 = self.scan_ave[0,10]
            side= self.scan_ave[0,9]
            b60 = self.scan_ave[0,8]
            b30 = self.scan_ave[0,7]
            bo  = self.scan_ave[0,5]
            sign_x = 1
            sign_rot = -1
        else:
            fr  = self.scan_ave[0,6]
            f30 = self.scan_ave[0,7]
            f60 = self.scan_ave[0,8]
            side= self.scan_ave[0,9]
            b60 = self.scan_ave[0,10]
            b30 = self.scan_ave[0,11]
            bo  = self.scan_ave[0,1]
            sign_x = -1
            sign_rot = 1

        ratiof = f30 / side
        ratiob = b30 / side
        print "Lider", '{:.0f}'.format(fr), '{:.0f}'.format(f30), '{:.0f}'.format(f60), '{:.0f}'.format(side), '{:.0f}'.format(b60), '{:.0f}'.format(b30),
        print "Dir", '{:.3f}'.format(ratiof), '{:.3f}'.format(ratiob), 
        ret = 0
        if fr < 110:
            x = -0.1
            th = 0
        elif fr < 200 or f30 < 160:
            x = 0
            #th = 2.0
            th = 0
            ret = 1
        elif b60 < side:
            x = 0
            th = 0.5
        elif f60 < side:
            x = 0
            th = -0.5
        else:
            x = 0.22
            if ratiof > 3.0 or ratiof < 1.333:
                ratiof = 2.0
            if ratiob > 3.0 or ratiob < 1.333:
                ratiob = 2.0
            if side > 180:
                if ratiof > 1.76 or ratiob < 2.34:
                    th = 0.2
                else:
                    th = 0
            elif side > 150:
                if ratiof > 2.1 or ratiob < 1.9:
                    th = 0.2
                else:
                    th = 0
            elif side > 120:
                if ratiof < 1.9 or ratiob > 2.1:
                    th = -0.2
                else:
                    th = 0
            else:
                if ratiof < 2.34 or ratiob > 1.76:
                    th = -0.2
                else:
                    th = 0
	    if bo > 200 and bo < 300:
		x = 0
		th = 0
		ret = 1

        twist = Twist()
        twist.linear.x = x * sign_x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th * sign_rot
        print "Twist", '{:.3f}'.format(x), '{:.3f}'.format(th)
        #print " myPos", '{:.3f}'.format(self.myPosX), '{:.3f}'.format(self.myPosY), '{:.3f}'.format(self.myDirect)
        #print " myPos", self.myPosX, self.myPosY, self.myDirect
        self.vel_pub.publish(twist)
        #return twist
        return ret

    def strategy(self):
        r = rospy.Rate(3) # change speed 3fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # ---> testrun
        #while not rospy.is_shutdown():
        #    NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
        #    _x = NextGoal_coor[0]
        #    _y = NextGoal_coor[1]
        #    _th = NextGoal_coor[2] * DEGRAD
        #    ret = self.setGoal(_x, _y, _th)
        #    self.basic_mode_process_step_idx += 1
        #    if self.basic_mode_process_step_idx >= len(basic_coordinate):
        #        self.basic_mode_process_step_idx = 0
        # ---< testrun
        mode = 0
        zone = 2
        direction = 0 
        while not rospy.is_shutdown():
	    print 'mode=',mode,'zone =',zone, "step_idx=", self.basic_mode_process_step_idx
            if mode == 0:
		NextGoal_coor = target_coordinate[zone, self.basic_mode_process_step_idx ]
		_x = NextGoal_coor[0]
		_y = NextGoal_coor[1]
		_th = NextGoal_coor[2] * DEGRAD
		ret = self.setGoal(_x, _y, _th)
		self.basic_mode_process_step_idx += 1
		#if self.basic_mode_process_step_idx >= 5:
		if self.basic_mode_process_step_idx >= 4:
		    self.basic_mode_process_step_idx = 0
		    if zone == 0:
			zone = 3
		    else:
			zone -= 1
		    mode = 1
            elif mode == 1:
                #print 'direction =', direction
                ret = self.calcTwist(direction)
                if ret == 1:
                    #if direction == 3:
                    #    direction = 0
                    #else:
                    #    direction += 1
                    mode = 0

            #print(twist)
            #self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

