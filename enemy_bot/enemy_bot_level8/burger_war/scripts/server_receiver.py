#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Import
import rospy
import json
import tf
import copy
import numpy as np
from math import cos

#Import ROS message type
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16MultiArray, Int8, Bool
from sensor_msgs.msg import LaserScan

class Target(object):
    def __init__(self,position):
        self.name = "n"
        self.position = position
        self.time = 0

class ServerReceiver(object):
    def __init__(self):
        #Get parameter
        self.side = rospy.get_param("~side", "r")
        self.enemy_side = "b" if self.side =="r" else "r"
        self.near_backwall_dist = rospy.get_param("~near_backwall_dist",0.28)
        self.near_frontwall_dist = rospy.get_param("~near_frontwall_dist",0.20)
        current_dir = rospy.get_param("~current_dir")

        #Initialize wall target states
        with open(current_dir+'/marker_pose.json') as f:
            self.wall_target_states = json.load(f)

        #Initialize enemy target states
        if self.side == "r":
            self.enemy_target_states = {"BL_B":"n","BL_R":"n","BL_L":"n"}
        else:
            self.enemy_target_states = {"RE_B":"n","RE_R":"n","RE_L":"n"}

        #Copy previous target state
        self.wall_target_states_pre = copy.deepcopy(self.wall_target_states)

        #Initialize robot position
        self.enemy_pose = PoseStamped()
        self.my_pose = PoseStamped()
        if self.side == "r":
            self.enemy_pose.pose.position = Point(1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.enemy_pose.pose.position = Point(-1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        else:
            self.enemy_pose.pose.position = Point(-1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.my_pose.pose.position = Point(1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        #Initialize other variables
        self.passed_time = 0
        self.color_flag = [0,0,0,0,0,0]
        self.lidar_flag = False #敵がLidarで見えるかどうか
        self.succeeded_goal = False
        self.near_backwall = False
        self.enemy_catch_time = 0

        #Subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
        self.enemy_pose_sub = rospy.Subscriber('absolute_pos',PoseStamped,self.enemyposeCallback)
        self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped,self.myposeCallback)
        self.color_flag_sub = rospy.Subscriber('color_flag_time',Int16MultiArray, self.colorCallback)
        self.lidar_flag_sub = rospy.Subscriber('lidar_flag',Bool, self.lidarCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarDataCallback)

    #Update target information
    def target_player_update(self,target_data):
        for info in target_data:
            target_name = info.get("name")
            if target_name in self.wall_target_states:
                self.wall_target_states_pre[target_name]["player"] = self.wall_target_states[target_name]["player"]
                self.wall_target_states[target_name]["player"] = info.get("player")
            elif target_name in self.enemy_target_states:
                self.enemy_target_states[target_name] = info.get("player")
    
    #Callback method
    def serverCallback(self, data):
        server_data = json.loads(data.data)
        target_info = server_data["targets"]
        self.target_player_update(target_info)
        self.passed_time = server_data["time"]

    def myposeCallback(self,pose):
        self.my_pose = pose

    def enemyposeCallback(self, pose):
        self.enemy_catch_time = rospy.Time.now().to_sec()
        self.enemy_pose = pose

    def colorCallback(self, array):
        self.color_flag = array.data
    
    def lidarCallback(self, data):
        self.lidar_flag = data.data

    def lidarDataCallback(self,data):
        self.near_frontwall = False
        self.near_backwall = False
        #前方が障害物と近いかチェック
        f_scan = data.ranges[-10:]+data.ranges[:10]
        f_scan = [x for x in f_scan if x > 0.1]
        if min(f_scan) < self.near_frontwall_dist / cos(5/180*np.pi):
            self.near_frontwall = True
        f_scan = data.ranges[-20:-10]+data.ranges[10:20]
        f_scan = [x for x in f_scan if x > 0.1]
        if min(f_scan) < self.near_frontwall_dist / cos(15/180*np.pi):
            self.near_frontwall = True
        f_scan = data.ranges[-30:-20]+data.ranges[20:30]
        f_scan = [x for x in f_scan if x > 0.1]
        if min(f_scan) < self.near_frontwall_dist / cos(25/180*np.pi):
            self.near_frontwall = True
        f_scan = data.ranges[-40:-30]+data.ranges[30:40]
        f_scan = [x for x in f_scan if x > 0.1]
        if min(f_scan) < self.near_frontwall_dist / cos(35/180*np.pi):
            self.near_frontwall = True
        f_scan = data.ranges[-50:-40]+data.ranges[40:50]
        f_scan = [x for x in f_scan if x > 0.1]
        if min(f_scan) < self.near_frontwall_dist / cos(45/180*np.pi):
            self.near_frontwall = True
        #後方が障害物と近いかチェック
        b_scan = data.ranges[170:190]
        b_scan = [x for x in b_scan if x > 0.1]
        if min(b_scan) < self.near_backwall_dist / cos(5/180*np.pi):
            self.near_backwall = True
        b_scan = data.ranges[160:170]+data.ranges[190:200]
        b_scan = [x for x in b_scan if x > 0.1]
        if min(b_scan) < self.near_backwall_dist / cos(15/180*np.pi):
            self.near_backwall = True
        b_scan = data.ranges[150:160]+data.ranges[200:210]
        b_scan = [x for x in b_scan if x > 0.1]
        if min(b_scan) < self.near_backwall_dist / cos(25/180*np.pi):
            self.near_backwall = True
        b_scan = data.ranges[145:150]+data.ranges[210:215]
        b_scan = [x for x in b_scan if x > 0.1]
        if min(b_scan) < self.near_backwall_dist / cos(32.5/180*np.pi):
            self.near_backwall = True
        #b_scan = data.ranges[140:150]+data.ranges[210:220]
        #b_scan = [x for x in b_scan if x > 0.1]
        #if min(b_scan) < self.near_backwall_dist / cos(35/180*np.pi):
            #self.near_backwall = True
        #b_scan = data.ranges[135:140]+data.ranges[220:225]
        #b_scan = [x for x in b_scan if x > 0.1]
        #if min(b_scan) < self.near_backwall_dist / cos(42.5/180*np.pi):
            #self.near_backwall = True

    #Choose target
    def nearest_target(self):
        target_list = [ d for d in self.wall_target_states if not self.wall_target_states[d].get("player") == self.side]
        candidate_name = target_list[0]
        min_dist = self.target_distance(candidate_name)
        for target_name in target_list:
            dist = self.target_distance(target_name)
            if dist < min_dist:
                candidate_name = target_name
                min_dist = dist
        return candidate_name

    def nearest_taken_target(self):
        target_list = [ d for d in self.wall_target_states if self.wall_target_states[d].get("player") == self.enemy_side]
        if not target_list:
            return self.nearest_target()
        else:
            candidate_name = target_list[0]
            min_dist = self.target_distance(candidate_name)
            for target_name in target_list:
                dist = self.target_distance(candidate_name)
                if dist < min_dist:
                    candidate_name = target_name
                    min_dist = dist
        return candidate_name

    def enemy_far_target(self):
        target_list = [ d for d in self.wall_target_states if not self.wall_target_states[d].get("player") == self.side]
        candidate_name = target_list[0]
        max_dist = self.target_distance_from_enemy(candidate_name)
        for target in target_list:
            dist = self.target_distance_from_enemy(target)
            if dist > max_dist:
                candidate_name = target
                max_dist = dist
        return candidate_name
        
    #Calculate distance
    def target_distance(self, target_name):
        diff_x = self.wall_target_states[target_name]["pose"][0]-self.my_pose.pose.position.x
        diff_y = self.wall_target_states[target_name]["pose"][1]-self.my_pose.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    def enemy_distance(self):
        diff_x = self.enemy_pose.pose.position.x - self.my_pose.pose.position.x
        diff_y = self.enemy_pose.pose.position.y - self.my_pose.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    def target_distance_from_enemy(self, target_name):
        diff_x = self.wall_target_states[target_name]["pose"][0]-self.enemy_pose.pose.position.x
        diff_y = self.wall_target_states[target_name]["pose"][1]-self.enemy_pose.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    #Debug method
    def show_wall_target_states(self):
        print("{}".format(json.dumps(self.wall_target_states,indent=4)))
        
    def show_distnce(self):
        for target_name in self.wall_target_states:
            print target_name, self.wall_target_states[target_name]["distance"]

    def show_pose(self):
        for target_name in self.wall_target_states:
            print target_name, self.wall_target_states[target_name]["pose"]