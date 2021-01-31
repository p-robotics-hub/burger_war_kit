#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Import
import rospy
import json
import numpy as np
import tf
import sys
import os
import actionlib
import copy
import time
import math

#Import ROS topic type
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Pose
from std_msgs.msg import String, Int16MultiArray, Int8, ColorRGBA
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Import ROS service type
from burger_war_level8.srv import DesiredPose, VisualFeedbackFlag
from std_srvs.srv import Empty,EmptyResponse

#Import other program
import def_state
from server_receiver import ServerReceiver
        
class SendPriorityGoal(ServerReceiver): #ServerReceiverの継承
    def __init__(self):
        #print("init!!!!!")
        super(SendPriorityGoal,self).__init__()
        #Get parameter
        self.robot_namespace = rospy.get_param("~robot_namespace", '')
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.80)
        self.enemy_distance_th_small = rospy.get_param("~enemy_distance_th_small",0.50)
        self.enemy_close = rospy.get_param("~enemy_close", 0.2)
        self.escape_distance = rospy.get_param("~escape_distance", 0.6)
        self.time_th = rospy.get_param("~time_th", 150)
        self.close_th = rospy.get_param("~close_th",0.8)
        self.update_enemy_time = rospy.get_param("~update_enemy_time", 2.0)
        self.stack_time = rospy.get_param("~stack",5.0)
        self.enemy_find_time = rospy.get_param("~enemy_find_time",13.0)
        self.enemy_target_offset = rospy.get_param("~enemy_target_offset",0.1)
        
        #Service Server
        self.path_success_srv = rospy.Service("pathplan_succeeded",Empty, self.successCallback)

        #Service Client
        #print("send priority goal init A!!!!!")
        rospy.wait_for_service("desired_pose")
        self.desired_pose_call = rospy.ServiceProxy("desired_pose",DesiredPose)
        #print("send priority goal init B!!!!!")
        rospy.wait_for_service("reset_pathplan")
        self.cancel_goal_call = rospy.ServiceProxy("reset_pathplan", Empty)
        #print("send priority goal init C!!!!!")
        rospy.wait_for_service("vf_flag")
        self.vf_flag_call = rospy.ServiceProxy("vf_flag", VisualFeedbackFlag) 

        #Action Client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()

        #Publisher for rviz
        self.goal_pub = rospy.Publisher("desired_goal",PoseStamped, queue_size=1)

        #Velocity dealer
        self.vel_sub = rospy.Subscriber('cmd_vel',  Twist, self.velCallback)
        self.pre_vel_time = rospy.Time.now().to_sec() + 5 #はじめ5秒間はEscapeしない

        #Initialize other variables
        self.goal_reached = False
        self.initial_end_dist = self.enemy_distance_th #この距離まで敵に近づくまでは順番に取る
        self.initial_state = True 
        self.target_number = 0 
#        self.target_order = [[-0.951,-0.33,-np.pi/6],[-0.471,-0.2,np.pi/6],
#            [0,-0.53,np.pi],[0,-0.53,np.pi/2],[0,-0.53,0]] #的を取る順番      # red
        self.target_order = [[0.951,0.33,np.pi-np.pi/6],[0.471,0.2,np.pi+np.pi/6],
            [0,0.53,np.pi-np.pi],[0,0.53,np.pi-np.pi/2],[0,-.53,np.pi]] #的を取る順番       # blue
        self.escape_flag = [0,0] #両方Trueの時に敵から遠い的を取りに行く 1:vf状態に入ったか 2:escape状態に入ったか
        #print("init end!!!!!")

    def velCallback(self, data):
        if not (data.linear.x == 0.0 and data.angular.z == 0.0):
            self.pre_vel_time = rospy.Time.now().to_sec()

    def successCallback(self,req):
        self.goal_reached = True
        return EmptyResponse()

    def send_goal(self,target):
        goal = PoseStamped()        
        goal.header.frame_id = self.robot_namespace + "/map"
        #print("send goal start !!!!!!")
        if target == "Enemy":
            th = math.atan2(self.enemy_pose.pose.position.y-self.my_pose.pose.position.y,
                            self.enemy_pose.pose.position.x-self.my_pose.pose.position.x)
            x = self.enemy_pose.pose.position.x - self.enemy_target_offset*math.cos(th)
            y = self.enemy_pose.pose.position.y - self.enemy_target_offset*math.sin(th)
            goal.pose.position.x = x
            goal.pose.position.y = y
            q = tf.transformations.quaternion_from_euler(0,0,th)
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        elif type(target) is unicode or type(target) is str:
            goal.pose.position.x = self.wall_target_states[target]["pose"][0]
            goal.pose.position.y = self.wall_target_states[target]["pose"][1]
            q = tf.transformations.quaternion_from_euler(0,0,self.wall_target_states[target]["pose"][2])
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        elif type(target) is list:
            goal.pose.position.x = target[0]
            goal.pose.position.y = target[1]
            q = tf.transformations.quaternion_from_euler(0,0,target[2])
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        #print("send goal end!!!!!!")
        self.goal_pub.publish(goal) #for rviz
        self.desired_pose_call(goal)

    def vf(self,previous_state,target):
        pre_state = previous_state
        if self.color_flag[2] and self.enemy_distance() < self.vf_B_dist:
            self.escape_flag[0] = 1
            if not pre_state == "Visual_Feedback":
                self.cancel_goal_call()
                self.vf_flag_call(Int8(data=2))
                pre_state = "Visual_Feedback"
                self.show_state_and_target(pre_state)

            if self.color_flag[3]: #AR読み取れたら
                print "AR読み取り成功"
                self.vf_flag_call(Int8(data=4))
                pre_state = ""

        if pre_state =="Visual_Feedback":
            if (not self.color_flag[2]) or self.enemy_distance < self.enemy_close or \
                 self.near_frontwall: #マーカーが見えなくなるor敵と近づきすぎるor壁と近づきすぎる
                print "AR読み取り失敗"
                self.near_frontwall = False
                self.vf_flag_call(Int8(data=4))
                pre_state = ""

        return pre_state
    
    def show_state_and_target(self,state,target=""):
        print "\n【State】",state
        if not target == "":
            print "【Target】",target
        print ""

    def main(self):

        print("main start!!!!!!")
        pre_state = ""
        target = ""
        send_time = 0
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            enemy_dist = self.enemy_distance()

            #初期Stateを抜ける条件
            if self.initial_state and enemy_dist < self.initial_end_dist and self.passed_time > self.enemy_find_time:
                self.initial_state = False
            #敵との距離が近いか
            ene_is_near = True if enemy_dist < self.enemy_distance_th else False
            ene_is_near_small = True if enemy_dist < self.enemy_distance_th_small else False
            #敵との距離が近すぎるか
            ene_is_close = True if enemy_dist < self.enemy_close else False
            #相手が見えているかどうか
            find_enemy = True if self.color_flag[2] else False
            #時間が残っているか
            in_time = True if self.passed_time < self.time_th else False
            #かなり近いところに壁のマーカーがあるか
            nearset_target = self.nearest_target()
            nearest_dist = self.target_distance(nearset_target)
            marker_is_close = True if nearest_dist < self.close_th else False
            no_marker_close = True if not marker_is_close else False
            #敵の位置を更新するか
            try: #targetがlistの場合もあるので
                if target in self.enemy_target_states and rospy.Time.now().to_sec() - send_time > self.update_enemy_time:
                    update_enemy = True
                else:
                    update_enemy = False
            except:
                pass
            #敵の位置が一定時間以上送られてこない時、敵の位置をリセット(遠くへ飛ばす)
            now = rospy.Time.now().to_sec()
            if now - self.enemy_catch_time > 12:
                self.enemy_pose.pose.position.x = 3.0
                self.enemy_pose.pose.position.y = 3.0

            #vf_flag_call->0:vfしない、1:壁のマーカーを取りに行くvf_A→廃止、2:敵のマーカーと取りに行くvf_B、3:回避 vf_C, 4:敵を向く, -1:確実に止まる 
            #color_flag->0:赤い風船、1:壁のマーカー、2:敵のマーカー、3:敵のAR、4:壁のAR

            #敵の的を2個以上取っていない状態ならすぐに敵の方へvf
            count = 0
            for enemy_target in self.enemy_target_states:
                if self.enemy_target_states[enemy_target] == self.side :
                    count += 1
            if count ==0:
                self.vf_B_dist = 1.2
            elif count ==1:
                self.vf_B_dist = 0.8
            elif count ==2:
                self.vf_B_dist = 0.7
            elif count == 3:
                self.vf_B_dist = 0.0 # vf しない

            vel_time_diff = rospy.Time.now().to_sec() - self.pre_vel_time

            if self.initial_state:
                #決まった順番で壁の的を取りに行く
                if not pre_state == "Go_to_the_marker_in_order" :
                    target = self.target_order[self.target_number]
                    self.send_goal(target)
                    send_time = rospy.Time.now().to_sec()
                    time.sleep(0.5)
                    pre_state = "Go_to_the_marker_in_order"
                    self.show_state_and_target(pre_state,target)
                
                if pre_state == "Go_to_the_marker_in_order":
                    if self.goal_reached:
                        print "ゴール到達"
                        self.goal_reached = False
                        pre_state = ""
                        self.target_number += 1
                        if self.target_number == len(self.target_order):
                            self.initial_state = False

                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)
            
            #elif (ene_is_close and find_enemy) or vel_time_diff > self.stack_time: #or self.color_flag[3]: 
            elif ene_is_close or vel_time_diff > self.stack_time: #or self.color_flag[3]: 
                #回避
                self.escape_flag[1] = 1
                if not pre_state =="Escape":
                    self.cancel_goal_call()
                    #self.vf_flag_call(Int8(data=4))
                    self.vf_flag_call(Int8(data=3))
                    pre_state = "Escape"
                    self.show_state_and_target(pre_state)
                    while not rospy.is_shutdown():
                        if self.enemy_distance() > self.enemy_distance_th or self.near_backwall:
                            self.vf_flag_call(Int8(data=-1))
                            pre_state = ""
                            break
                        r.sleep()
            
            elif (ene_is_near and self.lidar_flag) or ene_is_near_small:
                #相手の方を向く
                if not pre_state =="Visual_Feedback":
                    if (not pre_state == "Turn_to_the_enemy" ) or update_enemy:
                        self.cancel_goal_call()
                        self.vf_flag_call(Int8(data=4))
                        pre_state = "Turn_to_the_enemy"
                        self.show_state_and_target(pre_state)

                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)

            elif in_time and no_marker_close:
                if all(self.escape_flag):
                    #敵から遠いところにある的を取りに行く
                    if not pre_state =="Visual_Feedback":
                        if (not pre_state == "Go_to_the_marker_far_from_enemy" ) or update_enemy:
                            self.vf_flag_call(Int8(data=0))
                            target = self.enemy_far_target()
                            self.send_goal(target)
                            send_time = rospy.Time.now().to_sec()
                            time.sleep(0.5)
                            pre_state = "Go_to_the_marker_far_from_enemy"
                            self.show_state_and_target(pre_state,target)

                    if pre_state == "Go_to_the_marker_far_from_enemy":
                        if self.goal_reached or self.wall_target_states[target]["player"]==self.side:
                            self.escape_flag = [0,0]
                            self.cancel_goal_call()
                            print "ゴール到達"
                            self.goal_reached = False
                            self.escape = False
                            pre_state = ""
                    #条件がそろえばvf
                    pre_state = self.vf(pre_state,target)
                elif count < 2:
                    #敵の的を取りに行く
                    if not pre_state =="Visual_Feedback":
                        if (not pre_state == "Go_to_the_enemy_marker" ) or update_enemy:
                            self.vf_flag_call(Int8(data=0))
                            target = "Enemy"
                            self.send_goal(target)
                            send_time = rospy.Time.now().to_sec()
                            time.sleep(0.5)
                            pre_state = "Go_to_the_enemy_marker"
                            self.show_state_and_target(pre_state,target)

                    if pre_state == "Go_to_the_enemy_marker":
                        if self.goal_reached:
                            self.cancel_goal_call()
                            print "ゴール到達"
                            self.goal_reached = False
                            pre_state = ""
                    #条件がそろえばvf
                    pre_state = self.vf(pre_state,target)
                else:
                    #最も高い壁の的（相手が取った的）を取りに行く
                    if not pre_state =="Visual_Feedback":
                        if (not pre_state == "Go_to_the_taken_marker" ) or update_enemy:
                            self.vf_flag_call(Int8(data=0))
                            target = self.nearest_taken_target()
                            self.send_goal(target)
                            send_time = rospy.Time.now().to_sec()
                            time.sleep(0.5)
                            pre_state = "Go_to_the_taken_marker"
                            self.show_state_and_target(pre_state,target)

                    if pre_state == "Go_to_the_taken_marker":
                        if self.goal_reached or self.wall_target_states[target]["player"]==self.side:
                            self.escape_flag = [0,0]
                            self.cancel_goal_call()
                            print "ゴール到達"
                            self.goal_reached = False
                            pre_state = ""
                    #条件がそろえばvf
                    pre_state = self.vf(pre_state,target)

            else:
                #最も近い的を取りに行く
                if not pre_state =="Visual_Feedback":
                    if (not pre_state == "Go_to_the_nearest_marker" ) or update_enemy:
                        self.vf_flag_call(Int8(data=0))
                        target = self.nearest_target()
                        self.send_goal(target)
                        send_time = rospy.Time.now().to_sec()
                        time.sleep(0.5)
                        pre_state = "Go_to_the_nearest_marker"
                        self.show_state_and_target(pre_state,target)
                
                if pre_state == "Go_to_the_nearset_marker":
                    if self.goal_reached or self.wall_target_states[target]["player"]==self.side:
                        self.escape_flag = [0,0]
                        self.cancel_goal_call()
                        print "ゴール到達"
                        self.goal_reached = False
                        pre_state = ""

                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)
                r.sleep()
        

if __name__ == '__main__':
    print("start!!!!!")
    rospy.init_node('send_priority_goal__test')
    send = SendPriorityGoal()
    send.main()
