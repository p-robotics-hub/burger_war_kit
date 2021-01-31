#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError

from enum import Enum
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib_msgs
import json

# camera image 640*480
img_w = 640
img_h = 480
image_resize_scale = 8 # 8

fieldWidth = 170
fieldHeight = 170
centerBoxWidth = 35
centerBoxHeight = 35
otherBoxWidth = 20
otherBoxHeight = 15
otherBoxDistance = 53

# color definition
RED   = 1
GREEN = 2
BLUE  = 3

# target angle init value
COLOR_TARGET_ANGLE_INIT_VAL = -360
# distance to enemy init value
DISTANCE_TO_ENEMY_INIT_VAL = 1000

# PI
PI = 3.1415

# FIND_ENEMY status
FIND_ENEMY_SEARCH = 0
FIND_ENEMY_FOUND = 1
FIND_ENEMY_WAIT = 2
FIND_ENEMY_LOOKON = 3

# threshold
DISTANCE_KEEP_TO_ENEMY_THRESHOLD = 0.55 #1.5
DISTANCE_KEEP_TO_ENEMY_THRESHOLD_WHEN_LOWWER_SCORE = 0.55
DISTANCE_TO_WALL_THRESHOLD = 0.15
DISTANCE_TO_RED_COLOR_THRESHOLD = 1.00
ELAPSED_TIME_TO_ATTACK_ENEMY = 90 # (s)
F_IS_LOWWER_SCORE_THRESHOLD = 2

# robot running coordinate in BASIC MODE
basic_coordinate = np.array([
    # x, y, th
    [-0.9, 0.4, 0],   # 1
    [-0.9, -0.4, 0],  # 2
    [-0.9, 0.0, 0],   # 3
    [-0.4, 0.0, 0],   # 4
    [-0.3, 0.3, PI*1/4],# 5
    [0, 0.5, 0],      # 6
    [0, 0.5, PI*3/2], # 7
    [0, 0.5, PI],     # 8
    [0, 0.5, PI*7/4], # 9 
    [0.4, 0.15, PI*5/4],  # 10
#    [0.4, 0, PI],     # 10
#    [0.9, 0.4, PI],     # 
#    [0.9, -0.4, PI],    # 
    [0.3, -0.3, PI*5/4], # 11
    [0, -0.5, PI],    # 12
    [0, -0.5, PI/2],  # 13
    [0, -0.5, 0],     # 14
    [0, -0.5, PI],    # 15
    [-0.3, -0.3, PI*3/4], # 16
    [-0.4, 0.0, 0],   # 17
    [-0.9, 0.0, 0]    # 18
])

# enemy red circle distance table
enemyTable = [0.6,
              0.6150216460225,
              0.607633173466,
              0.600412110488,
              0.606233417988,
              0.614282558361667,
              0.622331698735333,
              0.631100177765,
              0.630015969276333,
              0.635059878230042,
              0.64010378718375,
              0.634263450900709,
              0.628423114617667,
              0.634487594167459,
              0.64055207371725,
              0.629659771919,
              0.649973412355,
              0.645224452019,
              0.6524137556555,
              0.6664434075355,
              0.661140998204584,
              0.655838588873667,
              0.678530305624,
              0.670010546843,
              0.67250341177,
              0.680896043777333,
              0.685414552689,
              0.691816230615,
              0.6815381348135,
              0.704492479563,
              0.69714076519,
              0.690572857857,
              0.715982198715,
              0.704492479563,
              0.719453021884,
              0.705150961876,
              0.7346112430095,
              0.728441655635667,
              0.7344124913215,
              0.733456492424,
              0.734174251556,
              0.742653310299,
              0.7413543760775,
              0.755296558142,
              0.7511338591575,
              0.761324495077,
              0.762299746275,
              0.772221475839667,
              0.771291062235834,
              0.770360648632,
              0.77670471370225,
              0.779124478499,
              0.796563923359,
              0.80022098124025,
              0.8122440874575,
              0.798520922661,
              0.808797210455,
              0.805208563805,
              0.826270878315,
              0.8229584296545,
              0.836533725262,
              0.8250925540925,
              0.843854486942333,
              0.8569021523,
              0.860339254141,
              0.84907412529,
              0.8710317760705,
              0.86767216026775,
              0.867370843887,
              0.8845794320106,
              0.872438232103667,
              0.884472489357,
              0.9022136429945,
              0.912724196911,
              0.90241180360325,
              0.916175325711667,
              0.924402515093667,
              0.926753371954,
              0.9307475209238,
              0.9475789666175,
              0.9519020318984,
              0.948839828372,
              0.960739517212,
              0.961377799511,
              0.9796719789506,
              0.982935011387,
              0.9911309679355,
              0.994026571513,
              1.0093265652668,
              1.0025129616255,
              1.029176020622,
              1.02375201384,
              1.02949428558333,
              1.03969413042,
              1.05134890760714,
              1.05352640152,
              1.05440887809,
              1.07356081009,
              1.078974747658,
              1.09659285015556,
              1.1049609979,
              1.11659020185667,
              1.121987342836,
              1.1275982558725,
              1.14080004692,
              1.15097039086286,
              1.16070830822,
              1.173641610144,
              1.178601121902,
              1.18465062976,
              1.203447914126,
              1.20636294782375,
              1.214876294136,
              1.23007002898571,
              1.23382780949333,
              1.24469544206286,
              1.26173377037,
              1.26935377121,
              1.2808631956575,
              1.29103107111714,
              1.30418515205286,
              1.30851991971167,
              1.3277992755175,
              1.342173020045,
              1.35351605074714,
              1.37383919954333,
              1.382196903228,
              1.39812651276625,
              1.40416569369286,
              1.43039790221571,
              1.43503899233571,
              1.45682920515375,
              1.47140628950857,
              1.48904720942222,
              1.50322542871714,
              1.51370613915571,
              1.5322475284325,
              1.54793909617857,
              1.55287077691889,
              1.58314231038125,
              1.59741815653727,
              1.618993282318,
              1.63261577818222,
              1.65953294436167,
              1.676973688602,
              1.691477441787,
              1.70667096701455,
              1.728128695487,
              1.753624777,
              1.779938717685,
              1.794945061207,
              1.81146056122111,
              1.84335380334077,
              1.87191375891333,
              1.88726128064462,
              1.91457062417727,
              1.93806938024692,
              1.96479229927067,
              1.99016849994533,
              2.01783529349714,
              2.05217889149733,
              2.07052585056857,
              2.10030040144938,
              2.14100282721944,
              2.17441822422833,
              2.20161106851389,
              2.23551858961563,
              2.27126028802556,
              2.30076575279222,
              2.34574483500667,
              2.38504777456684,
              2.4235638904572,
              2.46019279956654,
              2.5006335735325,
              2.5267824172985,
              2.519620728491
]

class ActMode(Enum):
    SEARCH = 1
    SNIPE  = 2
    ESCAPE = 3
    ATTACK = 4
    BASIC  = 5
    # (start) BASIC --> SNIPE --> SEARCH or ESCAPE or ATTACK --> (end)

class SeigoBot():
    myPosX = 0
    myPosY = -150
    myDirect = np.pi / 2
    lidarFig = plt.figure(figsize=(5,5))
    mapFig = plt.figure(figsize=(5,5))
    time_start = 0
    f_Is_lowwer_score = False
    f_isFrontBumperHit = False
    basic_mode_process_step_idx = 0 # process step in basic MODE
    search_mode_process_step_idx = -1 # process step in search MODE

    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        print("self.name", self.name)
        self.ImgDebug = False

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Lidar
        self.scan = LaserScan()
        topicname_scan = "scan"
        self.lidar_sub = rospy.Subscriber(topicname_scan, LaserScan, self.lidarCallback)
        self.front_distance = DISTANCE_TO_ENEMY_INIT_VAL # init
        self.front_scan = DISTANCE_TO_ENEMY_INIT_VAL
        self.back_distance = DISTANCE_TO_ENEMY_INIT_VAL # init
        self.back_scan = DISTANCE_TO_ENEMY_INIT_VAL

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        topicname_image_raw = "image_raw"
        self.image_sub = rospy.Subscriber(topicname_image_raw, Image, self.imageCallback)
        self.red_angle = COLOR_TARGET_ANGLE_INIT_VAL # init
        self.blue_angle = COLOR_TARGET_ANGLE_INIT_VAL # init
        self.green_angle = COLOR_TARGET_ANGLE_INIT_VAL # init
        self.red_distance = DISTANCE_TO_ENEMY_INIT_VAL # init

        # FIND_ENEMY status 
        self.find_enemy = FIND_ENEMY_SEARCH
        self.find_wait = 0
        self.enemy_direct = 1

        # joint
        self.joint = JointState()
        self.joint_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.moving = False

        # twist
        self.x = 0;
        self.th = 0;

        # war status
        topicname_war_state = "war_state"
	self.war_state = rospy.Subscriber(topicname_war_state, String, self.stateCallback)
        self.my_score = 0
        self.enemy_score = 0
        self.act_mode = ActMode.BASIC

        # odom
        topicname_odom = "odom"
	self.odom = rospy.Subscriber(topicname_odom, Odometry, self.odomCallback)

        # amcl pose
        topicname_amcl_pose = "amcl_pose"
	self.amcl_pose = rospy.Subscriber(topicname_amcl_pose, PoseWithCovarianceStamped, self.AmclPoseCallback)
        
        # time
        self.time_start = time.time()

    def odomCallback(self, data):
        # print(data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        e = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        # print(e[2] / (2 * np.pi) * 360)
        self.myDirect = e # rad

    def AmclPoseCallback(self, data):
        self.myPosX = data.pose.pose.position.x
        self.myPosY = data.pose.pose.position.y
        # print(self.myPosX, self.myPosY)

    def jointstateCallback(self, data):
        # Is moving?
        if np.abs(self.wheel_rot_r - data.position[0]) < 0.01 and np.abs(self.wheel_rot_l - data.position[1]) < 0.01:
            if self.moving is True:
                print "Stop!"
            self.moving = False
        else:
            if self.moving is False:
                print "Move!"
            self.moving = True

        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]

    def getElapsedTime(self):
        time_current = time.time()
        elapsed_time = time_current - self.time_start
        return elapsed_time

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

        # visualize scan data with radar chart
        angles = np.linspace(0, 2 * np.pi, len(self.scan.ranges) + 1, endpoint=True)
        values = np.concatenate((self.scan.ranges, [self.scan.ranges[0]]))
        ax = self.lidarFig.add_subplot(111, polar=True)
        ax.cla()
        ax.plot(angles, values, 'o-')
        ax.fill(angles, values, alpha=0.25)
        ax.set_rlim(0, 3.5)
        # self.front_distance = self.scan.ranges[0]
        self.front_distance = min(min(self.scan.ranges[0:10]),min(self.scan.ranges[350:359]))
        self.front_scan = (sum(self.scan.ranges[0:4])+sum(self.scan.ranges[355:359])) / 10
        self.back_distance = (min(self.scan.ranges[170:190]))
        self.back_scan = (sum(self.scan.ranges[176:185])) / 10

        # RESPECT @koy_tak        
	if (self.scan.ranges[0] != 0 and self.scan.ranges[0] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[10] != 0 and self.scan.ranges[10] < DISTANCE_TO_WALL_THRESHOLD) or (self.scan.ranges[350] != 0 and self.scan.ranges[350] < DISTANCE_TO_WALL_THRESHOLD):
            self.f_isFrontBumperHit = True
            print("self.f_isFrontBumperHit = True")
            self.cancelGoal()
        else:
            self.f_isFrontBumperHit = False

    def find_rect_of_target_color(self, image, color_type): # r:0, g:1, b:2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # red detection
        if color_type == RED:
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        # blue detection
        if color_type == BLUE:
            lower_blue = np.array([130, 50, 50])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # green detection
        if color_type == GREEN:
            lower_green = np.array([75, 50, 50])
            upper_green = np.array([110, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

        # neiborhood for dilate/erode
        neiborhood = np.array([[0, 1, 0],
                               [1, 1, 1],
                               [0, 1, 0]],
                              np.uint8)
        # dilate
        mask = cv2.dilate(mask,
                          neiborhood,
                          iterations=2)

        # erode
        mask = cv2.erode(mask,
                         neiborhood,
                         iterations=2)

        # get contours
        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects

    # camera image call back sample
    # convert image topic to opencv object and show
    def imageCallback(self, data):
        redFound = False
        greenFound = False

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        size = (img_w/image_resize_scale, img_h/image_resize_scale)
        frame = cv2.resize(self.img, size)
        # red
        rects = self.find_rect_of_target_color(frame, RED)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            if rect[3] > 2: # if red circle is enemy one (check if not noise)
                cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
                # angle(rad)
                tmp_angle = ((rect[0]+rect[0]+rect[2])/2-(img_w/(2*image_resize_scale)))*image_resize_scale *0.077
                self.red_angle = tmp_angle * np.pi / 180
                print ("red_angle", tmp_angle, self.red_angle)
                # distance (m)
                if image_resize_scale*rect[1] < len(enemyTable):
                    tmp_dist=enemyTable[image_resize_scale*rect[1]]
                    self.red_distance = tmp_dist if tmp_dist > 0 else 1
            else:
                self.red_distance = 1
                
            print ("red_angle, distance", self.red_angle, self.red_distance)
            # print ( tmp_angle )
            redFound = True
            self.trackEnemy(rect)
        else:
            self.red_angle = COLOR_TARGET_ANGLE_INIT_VAL

        # green
        rects = self.find_rect_of_target_color(frame, GREEN)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            if rect[3] > 1:
                cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 0), thickness=2)
                # angle(rad)
                tmp_angle = ((rect[0]+rect[0]+rect[2])/2-(img_w/(2*image_resize_scale)))*image_resize_scale *0.077
                self.green_angle = tmp_angle * np.pi / 180
                # print ("green_angle", tmp_angle, self.green_angle )
                # print ( tmp_angle )
                if  redFound is False:
                    greenFound = True
                    self.trackEnemy(rect)
        else:
            self.green_angle = COLOR_TARGET_ANGLE_INIT_VAL

        if redFound is False and greenFound is False:
            self.trackEnemy(None)

        # blue
        rects = self.find_rect_of_target_color(frame, BLUE)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)
            # angle(rad)
            tmp_angle = ((rect[0]+rect[0]+rect[2])/2-(img_w/(2*image_resize_scale)))*image_resize_scale *0.077
            self.blue_angle = tmp_angle * np.pi / 180
        else:
            self.blue_angle = COLOR_TARGET_ANGLE_INIT_VAL

        # either red/green color found, publish cancel topic
        if self.red_angle != COLOR_TARGET_ANGLE_INIT_VAL and self.green_angle != COLOR_TARGET_ANGLE_INIT_VAL:
            # if ElapsedTime is small, ignore enemy...
            if self.getElapsedTime() > ELAPSED_TIME_TO_ATTACK_ENEMY:
                # if red color is near position
                if self.red_distance < DISTANCE_TO_RED_COLOR_THRESHOLD:
                    self.cancelGoal()
            else:
                print("ignore enemy...", self.getElapsedTime() )
                self.red_distance = DISTANCE_TO_ENEMY_INIT_VAL

        if self.ImgDebug == True:
            cv2.imshow('image',frame)
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

        # update which bot is higher score
        if self.my_score <= ( self.enemy_score + F_IS_LOWWER_SCORE_THRESHOLD ):
            self.f_Is_lowwer_score = True
        else:
            self.f_Is_lowwer_score = False
            
        print("---")
        print("f_Is_lowwer_score", self.f_Is_lowwer_score)
        print("my_sore", self.my_score)
        print("enemy_score", self.enemy_score)
        print("elapsed_time", self.getElapsedTime())
        
    def approachToMarker(self):
        x = 0
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def keepMarkerToCenter(self, color_type, distance_threshold):

        # keep center to enemy
        angle = COLOR_TARGET_ANGLE_INIT_VAL
	if color_type == RED:
            angle = self.red_angle
	elif color_type == GREEN:
            angle = self.green_angle
	elif color_type == BLUE:
            angle = self.blue_angle
        else:
            print("invalid color_type", color_type)
            return

        if angle == COLOR_TARGET_ANGLE_INIT_VAL:
            return -1

        # keep distance to enemy when both red/green color found
        if self.red_angle != COLOR_TARGET_ANGLE_INIT_VAL and self.green_angle != COLOR_TARGET_ANGLE_INIT_VAL:
            _x = self.red_distance - distance_threshold
            
        else:
            _x = 0
            
        x = 0.2 * image_resize_scale * _x
        th = 3 * angle * (-1) # rad/s
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        self.vel_pub.publish(twist)
        time.sleep(0.500)
        
        return 0

    def trackEnemy(self, rect):
        # Found enemy
        if rect is not None:
            # Estimate the distance from enemy.
            if image_resize_scale*rect[1] < len(enemyTable):
                tmp_dist=enemyTable[image_resize_scale*rect[1]]
                d = tmp_dist if tmp_dist > 0 else 1
            else:
                d = 1
            # Estimate acceleration parameter
            d = d / (np.abs(rect[0] + rect[2] / 2.0 - img_w / 2.0) / float(img_w) * 4)
            # Decide the direction and the radian.
            if (rect[0] + rect[2] / 2.0) > img_w / 2.0:
                self.enemy_direct = -1 * d
            else:
                self.enemy_direct = 1 * d
            # Change state
            if self.find_enemy == FIND_ENEMY_SEARCH:
                # SEARCH->FOUND (Quick move)
                self.find_enemy = FIND_ENEMY_FOUND
                self.th = np.pi / 2.0 / self.enemy_direct
            else:
                # Maybe FOUND->LOOKON (Slow move)
                self.find_enemy = FIND_ENEMY_LOOKON
                self.th = np.pi / 16.0 / self.enemy_direct
            #print("Found enemy")

        else:
            # Lost enemy...
            # State change
            # LOOKON -> WAIT
            if self.find_enemy == FIND_ENEMY_LOOKON:
                self.find_wait = time.time()
                self.find_enemy = FIND_ENEMY_WAIT
                print("Wait for re-found")
            # WAIT -> SEARCH
            elif self.find_enemy == FIND_ENEMY_WAIT:
                if time.time() - self.find_wait > 10:
                    self.find_enemy = FIND_ENEMY_SEARCH
                    print("Start Search...")

            if self.find_enemy == FIND_ENEMY_SEARCH:
                # Search enemy
                # Default radian (PI/2)
                left_scan = np.pi / 2.0
                right_scan = np.pi / 2.0 * -1
                self.enemy_direct = self.enemy_direct / np.abs(self.enemy_direct)
                if self.scan.ranges is None or self.moving is True or self.front_distance == DISTANCE_TO_ENEMY_INIT_VAL:
                    # Now moving, do nothing!
                    print("Skip")
                    self.th = 0
                else:
                    # Is there something ahead?
                    if np.max(self.scan.ranges[0:5]) < 1.0 and np.max(self.scan.ranges[355:359]) < 1.0:
                        # Check both side, I do not watch the wall!
                        #print("Blind")
                        if self.enemy_direct > 0 and np.max(self.scan.ranges[80:100]) < 1.0:
                            self.enemy_direct = self.enemy_direct * -1
                        if self.enemy_direct < 0 and np.max(self.scan.ranges[260:280]) < 1.0:
                            self.enemy_direct = self.enemy_direct * -1
                    else:
                        # Calc radian
                        for i in range(5, 90, 1):
                            if np.max(self.scan.ranges[i - 5:i + 5]) < 0.7:
                                self.enemy_direct = self.enemy_direct * -1
                                break
                        left_scan = np.pi * i / 180.0
                        for i in range(354, 270, -1):
                            if np.max(self.scan.ranges[i - 5:i + 5]) < 0.7:
                                self.enemy_direct = self.enemy_direct * -1
                                break
                        right_scan = np.pi * (i - 360) / 180.0
                    # Set radian for twist
                    if self.enemy_direct < 0:
                        self.th = right_scan
                    else:
                        self.th = left_scan
            #print("Lost enemy")

    def Public_Twist_When_Bumper_Hit(self, _x, _th):
        twist = self.getTwist(_x, _th)
        self.vel_pub.publish(twist)
        time.sleep(0.500)
        
    def getTwist(self, _x, _th):
        twist = Twist()
        x = _x
        th = _th
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th        
        return twist

    def func_search_neighbourhood(self, p0, ps):
        L = np.array([])
        for i in range(ps.shape[0]):
            norm = np.sqrt( (ps[i][0] - p0[0])*(ps[i][0] - p0[0]) +
                            (ps[i][1] - p0[1])*(ps[i][1] - p0[1]) )
            L = np.append(L, norm)
        return np.argmin(L) ,ps[np.argmin(L)]

    def func_init(self):
        print("func_init")
        twist = Twist()
        r = rospy.Rate(1) # change speed fps
        time.sleep(1.000) # wait for init complete

        print("!!!!!! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! !!!!!!")
        print("!                                                     !")
        print("! do following command first for navigation           !")
        print("! $ rosservice call /gazebo/reset_simulation {}       !")
        print("!                                                     !")
        print("!!!!!! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! !!!!!!")
        print("")
        
        ElapsedTime = self.getElapsedTime()
        print("ElapsedTime",ElapsedTime)

        return

    def func_basic(self):
        print("func_basic")
        twist = Twist()

        # init search process
        if self.search_mode_process_step_idx < 0: # -1
            # get nearrest position
            current_coor = np.array([self.myPosX, self.myPosY])
            idx, nearrest_coor = self.func_search_neighbourhood(current_coor, basic_coordinate)
            print( idx, nearrest_coor[0], nearrest_coor[1], nearrest_coor[2] ) # idx, (x, y, th)
            self.search_mode_process_step_idx = idx
        elif self.basic_mode_process_step_idx >= len(basic_coordinate):
            self.basic_mode_process_step_idx = 0 

        # check front/back
        if self.f_isFrontBumperHit == True:
            print("f_isFrontBumperHit")
            self.Public_Twist_When_Bumper_Hit(-0.2, 0)

        # if red/green found, SNIPE mode
        if self.red_angle != COLOR_TARGET_ANGLE_INIT_VAL and self.green_angle != COLOR_TARGET_ANGLE_INIT_VAL:
            # if red color is near position
            if self.red_distance < DISTANCE_TO_RED_COLOR_THRESHOLD:
                self.act_mode = ActMode.ATTACK # transition to ATTACK
                return 0
        
        # basic
        NextGoal_coor = basic_coordinate[ self.basic_mode_process_step_idx ]
        _x = NextGoal_coor[0]
        _y = NextGoal_coor[1]
        _th = NextGoal_coor[2]
        ret = self.setGoal(_x, _y, _th)
        if ret == 0:
            self.basic_mode_process_step_idx += 1
            if self.basic_mode_process_step_idx >= len(basic_coordinate):
                self.basic_mode_process_step_idx = 0 
        else:
            print("setGoal ret:", ret)

        return 0

    def func_snipe(self):
        print("func_snipe")
        self.act_mode = ActMode.ATTACK
        return

    def func_search(self):
        print("func_search")
        self.act_mode = ActMode.BASIC
        return

    def func_escape(self):
        print("func_escape")        
        self.act_mode = ActMode.BASIC
        return

    def func_attack(self):
        print("func_attack")
        twist = Twist()

        cnt = 0
        rate=100
        r = rospy.Rate(rate) # change speed fps
        while not rospy.is_shutdown():
            # search 
            if cnt%2 == 0:
                twist = self.getTwist(0, 3.1415*2/3)
            else: # if cnt%2 == 1:
                twist = self.getTwist(0, -1*3.1415*2/3)
            self.vel_pub.publish(twist)

            # dog fight !!!
            for i in range(rate):

                # check front/back
                if self.f_isFrontBumperHit == True:
                    print("f_isFrontBumperHit")
                    self.Public_Twist_When_Bumper_Hit(-0.2, 0)

                # Is there something ahead?
                if np.max(self.scan.ranges[0:5]) < 0.4 and np.max(self.scan.ranges[355:359]) < 0.4:
                    distance_threshold = DISTANCE_KEEP_TO_ENEMY_THRESHOLD
                else:
                    distance_threshold = DISTANCE_KEEP_TO_ENEMY_THRESHOLD_WHEN_LOWWER_SCORE
                # keep enemy marker (RED/GREEN) to center position
                ret = self.keepMarkerToCenter(RED, distance_threshold)
                if ret == -1:
                    # if RED marker not found, keep GREEN color to center
                    ret = self.keepMarkerToCenter(GREEN, None)
                    if ret == -1:
                        # if lost enemy, transition to SEARCH BASIC mode
                        self.act_mode = ActMode.BASIC
                        self.search_mode_process_step_idx = -1
                        return

                r.sleep()
            cnt+=1

        return

    def strategy(self):
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # Main Loop --->
        self.func_init()
        r = rospy.Rate(100) # change speed fps

        # ---> testrun
        while not rospy.is_shutdown():

            print("act_mode: ", self.act_mode)

            if self.act_mode == ActMode.BASIC:
                self.func_basic()
            elif self.act_mode == ActMode.SNIPE: # SNIPE
                self.func_snipe()
            elif self.act_mode == ActMode.SEARCH:  # SEARCH
                self.func_search()
            elif self.act_mode == ActMode.ESCAPE: # ESCAPE
                self.func_escape()
            elif self.act_mode == ActMode.ATTACK: # ATTACK
                self.func_attack()
            else:
                print("act_mode: ", self.act_mode)

            r.sleep()
       # ---< testrun

if __name__ == '__main__':
    rospy.init_node('seigoRun')
    bot = SeigoBot('Seigo')
    bot.strategy()

