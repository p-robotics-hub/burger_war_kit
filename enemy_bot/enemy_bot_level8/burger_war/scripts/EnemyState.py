#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.
by Takuya Yamaguchi @dashimaki360
'''

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int16MultiArray
from burger_war_level8.srv import VisualFeedbackFlag,VisualFeedbackFlagResponse
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import tf
import numpy as np
from aruco_msgs.msg import MarkerArray
import requests
import json
from time import sleep

class EnemyBot(object):
    def __init__(self, use_camera=False):
        #Get parameter
        self.robot_namespace = rospy.get_param("~robot_namespace", 5)
        self.rate = rospy.get_param("~rate", 5)
        self.resi_per = rospy.get_param("~resize_rate", 0.8)

        #self.k_p_A_rot = 0.1 #pゲイン
        #self.k_i_A_rot = 0.0 #iゲイン
        #self.k_p_A_adv = 0.5 #pゲイン
        #self.k_i_A_adv = 0.0 #iゲイン
        self.k_p_B_rot = 0.2 #pゲイン
        self.k_i_B_rot = 0.0 #iゲイン
        self.k_p_B_adv = 0.65 #pゲイン
        self.k_i_B_adv = 0.0 #iゲイン
        #self.k_p_C_rot = 0.5 #0.2 #pゲイン
        self.k_p_esc = 0.5 #敵を向くときのpゲイン->回避
        self.k_i_esc = 0.0 #敵を向くときのiゲイン->回避
        self.k_p_adv = 1.0 # 敵から逃げる速度
        self.k_p_turn= 0.9 #敵を向くときのpゲイン->その場
        self.k_i_turn = 0.1 #敵を向くときのiゲイン->その場

        #self.diff_p_A_rot = 0
        #self.diff_i_A_rot = 0
        #self.diff_p_A_adv = 0
        #self.diff_i_A_adv = 0
        self.diff_p_B_rot = 0
        self.diff_i_B_rot = 0
        self.diff_p_B_adv = 0
        self.diff_i_B_adv = 0
        #self.diff_i_c = 0
        self.diff_i_esc = 0
        self.diff_i_turn = 0

        # カメラ画像上での赤マーカ位置,サイズ
        self.cam_Point_x = 0.0
        self.cam_Point_y = 0.0
        self.cam_Point_size = 0.0
        self.Red_Size_w = 0.0
        self.Red_Size_h = 0.0
        # 相手との相対位置
        self.Relative_Pose_x = 0.0
        self.Relative_Pose_y = 0.0
        # カメラ画像上でのARマーカ位置,サイズ
        self.cam_AR_x = 0.0
        self.cam_AR_y = 0.0
        self.AR_ID = 0
        self.cam_AR_size = 0.0
        # 相手の向き（ARより）
        self.AngleEnemy_AR = 0.0
        # 緑マーカ
        self.GreenCenter_X = 0
        self.GreenCenter_Y = 0
        self.Green_Size_w = 0
        self.Green_Size_h = 0
        self.GreenSize = 0.0
        self.Green_x = 0.0
        self.GreenNum = 0.0
        # 青マーカ
        self.BlueCenter_X = 0
        self.BlueCenter_Y = 0
        self.Blue_Size_w = 0
        self.Blue_Size_h = 0
        self.BlueSize = 0.0
        # target_id から取得したID一時保存
        self.real_target_id = 0
        # VFフラグ
        self.VF_change_Flag = 0
        self.VF_receive_time = 0

        # publisher
        self.relative_pose_pub = rospy.Publisher('relative_pose', PoseStamped ,queue_size=10)   
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.color_flag_pub = rospy.Publisher('color_flag', Int16MultiArray, queue_size=10)
        #self.image_pub = rospy.Publisher("cv_image",Image, queue_size=1)

        # service
        self.vf_flag_srv = rospy.Service("vf_flag", VisualFeedbackFlag, self.VFFlagCallback)

        self.my_pose = Pose()
        self.enemy_pose = Pose()

        # camera subscriber
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.target_id_sub = rospy.Subscriber('target_id', MarkerArray, self.targetIdCallback)
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped, self.myposeCallback)
            self.enemy_pose_sub = rospy.Subscriber('absolute_pos',PoseStamped, self.enemyposeCallback)
            #self.vf_flag_pub =rospy.Subscriber('/vf_flag', Int8, self.VFFlagCallback)
            
    def reset_diff(self):
        self.diff_p_B_rot = 0
        self.diff_i_B_rot = 0
        self.diff_p_B_adv = 0
        self.diff_i_B_adv = 0
        self.diff_i_c = 0
        self.diff_i_turn = 0

    def strategy(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            # 自分から見た敵の相対位置・向き
            temp_pose = PoseStamped()
            pose = PoseStamped()
            temp_pose.pose.position.y = self.Relative_Pose_y
            temp_pose.pose.position.x = self.Relative_Pose_x

            #euler_z = self.AngleEnemy_AR + 180*3.141592/180
            #　Gazebo座標からRviz座標
            euler_z = self.AngleEnemy_AR + np.pi
            #Gazebo座標からRviz座標
            pose.pose.position.x = temp_pose.pose.position.y
            pose.pose.position.y = -temp_pose.pose.position.x
            # オイラー角からクォータニオンへの変換
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_z)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # update twist
            twist = Twist()
            ########################以下、VisualFeedback#####################################

            #BlueマーカへのVF VF of A
            #if self.VF_change_Flag == 1:
                #self.diff_p_A_rot = (320*self.resi_per-self.BlueCenter_X) / (320*self.resi_per)
                #self.diff_i_A_rot += self.diff_p_A_rot
                #if math.fabs(self.diff_p_A_rot) *(320*self.resi_per) < 5:
                    #self.diff_i_A_rot = 0
                #twist.angular.z = self.k_p_A_rot*self.diff_p_A_rot + self.k_i_A_rot*self.diff_i_A_rot

                #self.diff_p_A_adv = (25000*self.resi_per*self.resi_per-self.BlueSize) / (25000*self.resi_per*self.resi_per)                
                #print(self.diff_p_A_adv)
                #self.diff_i_A_adv += self.diff_p_A_adv
                #if self.diff_p_A_adv < 0.1:
                    #self.diff_p_A_adv = 0.1
                    #self.diff_i_A_adv = 0
                #twist.linear.x = self.k_p_A_adv*self.diff_p_A_adv + self.k_i_A_adv*self.diff_i_A_adv
                #self.vel_pub.publish(twist)
                
            #GreenマーカへのVF VF of B
            if self.VF_change_Flag == 2:
                self.diff_p_B_rot = (320*self.resi_per-self.GreenCenter_X) / (320*self.resi_per)
                self.diff_i_B_rot += self.diff_p_B_rot
                if math.fabs(self.diff_p_B_rot) *(320*self.resi_per) < 5:
                    self.diff_i_B_rot = 0
                twist.angular.z = self.k_p_B_rot*self.diff_p_B_rot + self.k_i_B_rot*self.diff_i_B_rot

                self.diff_p_B_adv = (25000*self.resi_per*self.resi_per-self.GreenSize) / (25000*self.resi_per*self.resi_per)
                self.diff_i_B_adv += self.diff_p_B_adv
                if self.diff_p_B_adv < 0.1:
                    self.diff_p_B_adv = 0.1
                    self.diff_i_B_adv = 0.0
                twist.linear.x = self.k_p_B_adv*self.diff_p_B_adv + self.k_i_B_adv*self.diff_i_B_adv
                if math.fabs(self.diff_p_B_rot) > 0.5:
                    twist.linear.x = 0.1
                self.vel_pub.publish(twist)

            #回避
            elif self.VF_change_Flag == 3:
                #目標角速度算出
                target_th = math.atan2(self.enemy_pose.position.y-self.my_pose.position.y,
                                self.enemy_pose.position.x-self.my_pose.position.x)
                q = self.my_pose.orientation
                euler = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
                th = euler[2]
                diff_th = target_th - th
                if diff_th > math.pi:
                    diff_th -= math.pi
                elif diff_th < -math.pi:
                    diff_th += math.pi

                if diff_th < math.pi/9:
                    self.diff_i_esc = 0
                else:
                    self.diff_i_esc += diff_th
                twist.angular.z = self.k_p_esc*diff_th + self.k_i_esc * self.diff_i_esc
                #目標後速度算出
                now = rospy.Time.now().to_sec()
                diff = now - self.VF_receive_time
                twist.linear.x = -self.k_p_adv * diff
                if twist.linear.x < -1.5:
                    twist.linear.x = -1.5
                self.vel_pub.publish(twist)

            #相手の方を向く
            elif self.VF_change_Flag == 4:
                #目標角速度算出
                target_th = math.atan2(self.enemy_pose.position.y-self.my_pose.position.y,
                                self.enemy_pose.position.x-self.my_pose.position.x)
                q = self.my_pose.orientation
                euler = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
                th = euler[2]
                diff_th = target_th - th
                #角度丸め込み
                if diff_th > math.pi:
                    diff_th -= math.pi
                elif diff_th < -math.pi:
                    diff_th += math.pi

                if diff_th < math.pi/9:
                    self.diff_i_turn = 0
                else:
                    self.diff_i_turn += diff_th
                twist.angular.z = self.k_p_turn*diff_th + self.k_i_turn * self.diff_i_turn

                #時間がたったら敵に近づいて回避状態に入る
                now = rospy.Time.now().to_sec()
                diff = now - self.VF_receive_time
                if diff < 4:
                    twist.linear.x = 0
                else:
                    twist.linear.x = 0.01
                
                self.vel_pub.publish(twist)
            
            elif self.VF_change_Flag == 5:
                #VF_B と回避のつなぎ(滑り防止)
                twist.linear.x = 0.5
                #twist.angular.z = 0.0
                self.vel_pub.publish(twist)

            ###########################################################################

            # 相対位置・向きのpublish
            self.relative_pose_pub.publish(pose)

            # カメラ上にどのマーカみえているかのFlag
            self.ColorFlag = []
            if self.cam_Point_size > 0:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.BlueSize > 0:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.GreenSize > 1000: #あまり見えない時は深追いしない
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.AR_ID == 50 or self.AR_ID == 51 or self.AR_ID == 52:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.AR_ID > 0 and self.AR_ID < 50:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)

            ColorFlag_forPublish = Int16MultiArray(data=self.ColorFlag)
            self.color_flag_pub.publish(ColorFlag_forPublish)

            r.sleep()

    def myposeCallback(self,pose):
        self.my_pose = pose.pose

    def enemyposeCallback(self,pose):
        self.enemy_pose = pose.pose

    # target_IDを取得
    def targetIdCallback(self, data):
        markers = data.markers
        for marker in markers:
            self.real_target_id = str(marker.id)

    #赤色マーカの取得・敵相対位置計算
    def ColorCenter(self):
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        hsv_min = np.array([0,64,0])
        hsv_max = np.array([30,255,255])
        mask1 = cv2.inRange(hsv_img, hsv_min, hsv_max)
        hsv_min = np.array([150,64,0])
        hsv_max = np.array([179,255,255])
        mask2 = cv2.inRange(hsv_img, hsv_min, hsv_max)
        color_mask = mask1 + mask2
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0)
        size_max = 0
        size_max_x = 0
        size_max_y = 0
        size_max_w = 0
        size_max_h = 0
        center_max_x = 0
        center_max_y = 0      
        rela_pose_y= 0

        for i in range(1, nLabels):
            x, y, w, h, size = data[i]
            center_x, center_y = center[i]
            if size > size_max and center_y<240*self.resi_per:
                size_max_x = x
                size_max_y = y
                size_max_w = w
                size_max_h = h
                size_max = size
    #            center_max_x = size_max_x + size_max_w/2
    #            center_max_y = size_max_y + size_max_h/2
                center_max_x = center_x
                center_max_y = center_y

        if size_max < 100*self.resi_per*self.resi_per:        
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0

        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        
        return (center_max_x, center_max_y, size_max ,size_max_w,size_max_h )

    # 緑色マーカの認識
    def GreenColor(self):
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([30,64,150])
        color_max = np.array([90,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0,0.0)
        size_max = 0
        size_max_x = 0
        size_max_y = 0
        size_max_w = 0
        size_max_h = 0
        center_max_x = 0
        center_max_y = 0
        self.GreenNum = 0

        for i in range(1, nLabels):
            x, y, w, h, size = data[i]
            center_x, center_y = center[i]
            if size > 100*self.resi_per*self.resi_per:
                self.GreenNum += 1

            if size > size_max:
                size_max_x = x
                size_max_y = y
                size_max_w = w
                size_max_h = h
                size_max = size
                center_max_x = size_max_x + size_max_w/2
                center_max_y = size_max_y + size_max_h/2
    #            center_max_x = center_x
    #            center_max_y = center_y

        if size_max < 100*self.resi_per*self.resi_per:
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0
        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        
        return (center_max_x,center_max_y,size_max_w, size_max_h, size_max,size_max_x)

    # 青色マーカの認識
    def BlueColor(self):       
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([90,64,0])
        color_max = np.array([150,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0)
        size_max = 0
        size_max_x = 0
        size_max_y = 0
        size_max_w = 0
        size_max_h = 0
        center_max_x = 0
        center_max_y = 0

        for i in range(1, nLabels):
            x, y, w, h, size = data[i]
            center_x, center_y = center[i]
            if size > size_max:
                size_max_x = x
                size_max_y = y
                size_max_w = w
                size_max_h = h
                size_max = size
    #            center_max_x = size_max_x + size_max_w/2
    #            center_max_y = size_max_y + size_max_h/2
                center_max_x = center_x
                center_max_y = center_y

        if size_max < 100*self.resi_per*self.resi_per  or size_max_y < 100*self.resi_per:
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0
        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        
        return (center_max_x,center_max_y,size_max_w, size_max_h, size_max)

    def GetRelativePose(self):
        rela_pose_x = 0
        rela_pose_y = 0

        if self.GreenSize > 0:
            if self.Green_Size_h > 160:
                rela_pose_y= (0.0046*self.Green_Size_h*self.Green_Size_h-2.9342*self.Green_Size_h+749.72)/1000
            else:
                rela_pose_y= (0.0277*self.Green_Size_h*self.Green_Size_h-10.104*self.Green_Size_h+1307.7)/1000
            rela_pose_x = ((-1.8285*rela_pose_y + 0.2602)*(340*self.resi_per-self.GreenCenter_X)/self.resi_per +(-5.0838*rela_pose_y+0.5727)) / 1000
 
        if self.cam_Point_size > 0 and self.Red_Size_w<53:
            rela_pose_y= (0.0047*self.cam_Point_y/self.resi_per + 0.4775)
            if rela_pose_y > 1.0:
                #rela_pose_y= (0.0008*size_max*size_max/(self.resi_per*self.resi_per*self.resi_per*self.resi_per) - 2.3454*size_max/self.resi_per + 2764.9)/1000
                rela_pose_y= (0.0019*self.cam_Point_size*self.cam_Point_size - 3.6647*self.cam_Point_size + 2764.9)/1000 #resize 0.8
            rela_pose_x = ((-1.4104*rela_pose_y - 0.1011)*(340*self.resi_per-self.cam_Point_x)/self.resi_per +(21.627*rela_pose_y+7.827)) / 1000
            if self.cam_Point_size < 100*self.resi_per*self.resi_per:        
                rela_pose_x = 0
                rela_pose_y = 0
        
        return(rela_pose_x,rela_pose_y)

    #ARマーカのカメラ座標上での位置を取得
    def ARPointSearch(self):
        if self.real_target_id == 0:
            return (0,0,0,0)
            #
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.img, dictionary)
        #aruco.drawDetectedMarkers(self.img, corners, ids, (0,255,0))        
        if not corners:
            return (0,0,0,0)

        ARsize_max = 0
        ARcenter_max_x = 0
        ARcenter_max_y = 0
        ARsize = 0
        now_ID = 0
        for i in range(0, len(ids)):
            ARsize = (corners[i][0][1][0]-corners[i][0][0][0])*(corners[i][0][2][1]-corners[i][0][1][1])
            if ARsize_max < ARsize:
                if ARsize > 100:
                    ARcenter_max_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0])/4
                    ARcenter_max_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1])/4
                    ARsize_max = ARsize
                    now_ID = ids[i][0]
        enemy_angle = 0.0
        green_size = 0.0
        cv2.putText(self.img,str(now_ID),(70,100),cv2.FONT_HERSHEY_SIMPLEX, 3.0,(0, 0, 0),5)
        self.real_target_id = 0       
        
        return (ARcenter_max_x,ARcenter_max_y,ARsize_max,now_ID)

    def EnemyAngle(self):
        # 敵の向きを推定
        enemy_angle =0.0
        green_w_center_x , green_center_y ,green_wx , green_wy , green_size , green_x = self.GreenColor()
        if green_size==0:
            return(0.0)
        ttemp_theta = float(green_wx) / float(green_wy)
        temp_theta = -2.0964*ttemp_theta*ttemp_theta + 1.4861*ttemp_theta + 0.6648
        if ttemp_theta<0.5:
            temp_theta = -0.8613*ttemp_theta + 1.3752
        if self.cam_Point_size > 0 and self.GreenSize > 0:
            diffRG = self.cam_Point_x- self.GreenCenter_X
            if self.GreenNum == 1:
                if ttemp_theta > 0.95 and math.fabs(diffRG)<10.0:
                    if diffRG > 0:
                        enemy_angle = 180*3.141592/180 + temp_theta 
                        enemy_angle = enemy_angle - 360*3.141592/180
                    else:
                        enemy_angle = 180*3.141592/180 - temp_theta    
                else:
                    if ttemp_theta < 0.77:
                        if diffRG<0:
                            enemy_angle = 90*3.141592/180 - temp_theta
                        else:
                            
                            enemy_angle = 270*3.141592/180 + temp_theta 
                            enemy_angle = enemy_angle - 360*3.141592/180                   
                    else:
                        if diffRG>0:
                            enemy_angle = 90*3.141592/180 - temp_theta
                        else:
                            
                            enemy_angle = 270*3.141592/180 + temp_theta 
                            enemy_angle = enemy_angle - 360*3.141592/180
            elif self.GreenNum == 2:
                if math.fabs(diffRG)<10.0:
                    if diffRG > 0:
                        enemy_angle = 180*3.141592/180 + temp_theta 
                        enemy_angle = enemy_angle - 360*3.141592/180
                    else:
                        enemy_angle = 180*3.141592/180 - temp_theta
                else:
                    if diffRG > 0:
                        enemy_angle = 90*3.141592/180 + temp_theta                    
                    else:
                        enemy_angle = 270*3.141592/180 - temp_theta 
                        enemy_angle = enemy_angle - 360*3.141592/180

        if self.AR_ID == 50 or self.AR_ID == 51 or self.AR_ID == 52:
            PM_Flag = green_w_center_x - self.cam_AR_x
            if PM_Flag<-10*self.resi_per:
                green_wx = (self.cam_AR_x - green_x)*2
            elif PM_Flag>10*self.resi_per:
                green_wx = (green_x + green_wx - self.cam_AR_x)*2

            ttemp_theta = 0.0
            ttemp_theta = float(green_wx) / float(green_wy)
            temp_theta = -2.0964*ttemp_theta*ttemp_theta + 1.4861*ttemp_theta + 0.6648

            if self.AR_ID == 50:
                if PM_Flag > 0:
                    enemy_angle = 90*3.141592/180 + temp_theta 
                else:
                    enemy_angle = 90*3.141592/180 - temp_theta 
            elif self.AR_ID == 51:
                if PM_Flag > 0:
                    enemy_angle = 270*3.141592/180 + temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180
                else:
                    enemy_angle = 270*3.141592/180 - temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180

            elif self.AR_ID == 52:
                if PM_Flag > 0:
                    enemy_angle = 180*3.141592/180 + temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180
                else:
                    enemy_angle = 180*3.141592/180 - temp_theta    
        return(enemy_angle)

    def VFFlagCallback(self, data):
        self.VF_change_Flag = data.flag.data
        self.reset_diff()
        self.VF_receive_time = rospy.Time.now().to_sec()
        #STOP
        if self.VF_change_Flag ==-1:
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            self.vel_pub.publish(twist)
        return VisualFeedbackFlagResponse(True)

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = cv2.resize(self.img, dsize=None, fx=self.resi_per, fy=self.resi_per, interpolation=cv2.INTER_NEAREST)
        except CvBridgeError as e:
            rospy.logerr(e)

        self.cam_Point_x , self.cam_Point_y , self.cam_Point_size , self.Red_Size_w , self.Red_Size_h = self.ColorCenter()
        self.cam_AR_x , self.cam_AR_y , self.cam_AR_size , self.AR_ID= self.ARPointSearch()
        self.GreenCenter_X , self.GreenCenter_Y , self.Green_Size_w , self.Green_Size_h , self.GreenSize , self.Green_x = self.GreenColor()
        self.BlueCenter_X , self.BlueCenter_Y , self.Blue_Size_w , self.Blue_Size_h , self.BlueSize = self.BlueColor()
        self.Relative_Pose_x , self.Relative_Pose_y = self.GetRelativePose()
        self.AngleEnemy_AR = self.EnemyAngle()
        #print('AngleEnemy_AR' , self.AngleEnemy_AR*180/3.141592,float(self.Green_Size_w)/float(self.Green_Size_h),self.cam_Point_x- self.GreenCenter_X)
        #print(float(self.Green_Size_w)/float(self.Green_Size_h))
        #print('(x,y)' , self.Relative_Pose_x , self.Relative_Pose_y)
        #print('(Green x,y,h)=' , self.GreenCenter_X , self.GreenCenter_Y , self.Green_Size_h)
        #print('()=' , self.cam_AR_size,self.GreenSize)
        
        # for rviz
        #msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
        #self.image_pub.publish(msg)
        #rospy.sleep(0.001)

        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('relative_enemy_camera')
    bot = EnemyBot(use_camera=True)
    bot.strategy()
