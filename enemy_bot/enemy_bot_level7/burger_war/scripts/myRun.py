#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import rosparam
import tf
from math import radians, degrees, atan2
import tf2_ros

from geometry_msgs.msg import Twist, TransformStamped
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # RESPECT @seigot
import actionlib
from visualization_msgs.msg import Marker

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


def get_goals(my_color, rotation="CW"):
    if my_color == 'b':
        symbol = -1
        th = 180
    else:
        symbol = 1
        th = 0

    # rotation = 'CW'  # 回転方向を変える @en

    if rotation == 'CW':
        symbol_2 = -1
    else:
        symbol_2 = 1

    #print("get_goals", my_color, symbol, th, symbol_2)
        
    # 12x3 (x,y,yaw)
    TARGET = [
        [symbol*-0.8, symbol*symbol_2*0.4, radians(symbol_2*(10+th))],
        [symbol*-0.8, symbol*symbol_2*-0.4, radians(symbol_2*(-10+th))],
        [symbol*-0.5, symbol*symbol_2*0, radians(symbol_2*(0+th))],         # スタート地点正面のポイントを獲得
        [symbol*-0.5, symbol*symbol_2*0, radians(symbol_2*(-45+th))],
        [symbol*0, symbol*symbol_2*-0.5, radians(symbol_2*(180+th))],
        [symbol*0, symbol*symbol_2*-0.5, radians(symbol_2*(90+th))],
        [symbol*0, symbol*symbol_2*-0.5, radians(symbol_2*(0+th))],
        [symbol*0, symbol*symbol_2*-0.5, radians(symbol_2*(-45+th))],
        [symbol*0.4, symbol*symbol_2*-1.0, radians(symbol_2*(45+th))],
        [symbol*0.9, symbol*symbol_2*-0.55, radians(symbol_2*(180+th))],
        [symbol*0.9, symbol*symbol_2*-0.55, radians(symbol_2*(45+th))],
        [symbol*1.46, symbol*symbol_2*0, radians(symbol_2*(45+th))],  # top
        [symbol*1.46, symbol*symbol_2*0, radians(symbol_2*(135+th))], # top
        [symbol*0.9, symbol*symbol_2*0.6, radians(symbol_2*(180+th))],
        [symbol*0.9, symbol*symbol_2*0.6, radians(symbol_2*(135+th))],
        [symbol*0.4, symbol*symbol_2*1.0, radians(symbol_2*(135+th))],
        [symbol*0.4, symbol*symbol_2*1.0, radians(symbol_2*(-135+th))],
        [symbol*0, symbol*symbol_2*0.5, radians(symbol_2*(0+th))],
        [symbol*0, symbol*symbol_2*0.5, radians(symbol_2*(-90+th))],
        [symbol*0, symbol*symbol_2*0.5, radians(symbol_2*(180+th))],
        [symbol*0, symbol*symbol_2*0.5, radians(symbol_2*(-135+th))],
        [symbol*-0.5, symbol*symbol_2*0, radians(symbol_2*(0+th))],
    ]
    #print("TARGET[0]", TARGET[0])
    return TARGET


def num2mvstate(i):
    return ["PENDING", "ACTIVE", "RECALLED", "REJECTED", "PREEMPTED", "ABORTED", "SUCCEEDED", "LOST"][i]


class RandomBot():
    def __init__(self, bot_name):
        self.name = bot_name
        self.my_color = rospy.get_param('~rside')
        #print("enemy_mycolor=", self.my_color)
        self.robot_namespace = rospy.get_param('~robot_namespace')
        #print("enemy_robot_namespace=", self.robot_namespace)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.detect_inner_pub = rospy.Publisher('detect_inner_th', Marker,queue_size=1)
        self.detect_outer_pub = rospy.Publisher('detect_outer_th', Marker,queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(self.tfBuffer)
        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)  # RESPECT @seigot]
        self.goalcounter = 0
        self.goalcounter_prev = -1
        self.goals = get_goals(self.my_color)
        self.enemy_y = 0
        self.route_direction = None
        self.is_second_lap = False

    def setGoal(self, x, y, yaw):
        # RESPECT @seigot
        self.client.wait_for_server()
        #print('setGoal x=', x, 'y=', y, 'yaw=', yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.robot_namespace+'/map'
        # name = 'red_bot' if self.my_color == 'r' else 'blue_bot'
        # goal.target_pose.header.frame_id = name + '/map' if self.sim_flag == True else 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        def active_cb():
            # rospy.loginfo("active_cb. Goal pose is now being processed by the Action Server...")
            return

        def feedback_cb( feedback):
            #To print current pose at each feedback:
            #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
            # rospy.loginfo("feedback_cb. Feedback for goal pose received{}".format(feedback))
            return

        def done_cb(status, result):
            if status is not GoalStatus.PREEMPTED:
                self.goalcounter += 1
                if self.goalcounter == len(self.goals):
                    self.is_second_lap = True
                self.goalcounter %= len(self.goals)

                # 2周目の場合は、最初の方のマーカーをとばす
                if self.is_second_lap and (self.goalcounter in [0, 1, 2]):
                    self.goalcounter = 3
            rospy.loginfo("done_cb. status:{} result:{}".format(num2mvstate(status), result))

        self.client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        return
        # ret = self.client.send_goal_and_wait( goal, execute_timeout=rospy.Duration(4))
        # return ("PENDING", "ACTIVE", "RECALLED", "REJECTED", "PREEMPTED", "ABORTED", "SUCCEEDED", "LOST")[ret]

    def strategy(self):
        r = rospy.Rate(10)  # change speed 1fps

        self.setGoal(
            self.goals[0][0], self.goals[0][1], self.goals[0][2])

        is_patrol_mode_prev = False
        is_patrol_mode = True

        while not rospy.is_shutdown():
            r.sleep()
            # rospy.loginfo("is_patrol_mode:{}".format(is_patrol_mode) )

            # ルートの方向決定のループ
            if None == self.route_direction:
                # 敵の移動方向を検出
                map_name=self.robot_namespace+'/map'
                is_enemy_detected, x, y = self.getEnemyPos(map_name)
                if is_enemy_detected:
                    self.enemy_y += 1 if y > 0 else -1
                # スタート正面のポイントをとった時点で、移動方向を決定
                if 3 == self.goalcounter and 2 == self.goalcounter_prev:
                    if self.my_color == "r" and 0 >= self.enemy_y:
                        self.route_direction = "CW"
                    elif self.my_color == "r" and self.enemy_y > 0:
                        self.route_direction = "CCW"
                    elif self.my_color == "b" and 0 >= self.enemy_y:
                        self.route_direction = "CCW"
                    elif self.my_color == "b" and self.enemy_y > 0:
                        self.route_direction = "CW"
                    else:
                        assert()
                    self.goals = get_goals(self.my_color, self.route_direction)
                    #print("enemy_bot goals is... ", self.goals, self.my_color, self.route_direction)
                    
            # 敵の検出
            is_enemy_detected, enemy_dist, enemy_rad = self.getEnemyDistRad()
            # rospy.loginfo("is_enemy_detected:{} enemy_dist{} enemy_rad:{}".format(is_enemy_detected, enemy_dist, enemy_rad))

            # 敵追跡モードと巡回モードの分岐条件判定
            is_patrol_mode = True
            detect_inner_th = 0.5
            detect_outer_th = 0.6
            self.pubDetectRange(detect_inner_th, detect_outer_th)
            if not is_enemy_detected:
                is_patrol_mode = True
            elif is_patrol_mode and  detect_inner_th > enemy_dist:
                is_patrol_mode = False
            elif not is_patrol_mode and detect_outer_th < enemy_dist:
                is_patrol_mode = True

            # 移動実施
            if is_patrol_mode and (not is_patrol_mode_prev or (self.goalcounter is not self.goalcounter_prev)):
                # 新たに巡回モードに切り替わった瞬間及びゴール座標が変わった時
                # goalcounterのゴール座標をセット
                self.setGoal(self.goals[self.goalcounter][0], self.goals[self.goalcounter][1], self.goals[self.goalcounter][2])
                rospy.loginfo( num2mvstate(self.client.get_state()))
                self.goalcounter_prev = self.goalcounter
            elif is_patrol_mode:
                # 巡回モード最中。CBが来るまで何もしない。
                pass
            else : 
                # 敵の方向を向くモード
                self.client.cancel_all_goals()
                twist = Twist()
                twist.angular.z = radians(3.0*degrees(enemy_rad))
                self.vel_pub.publish(twist)
            is_patrol_mode_prev = is_patrol_mode

    def getEnemyPos(self, frame):
        try:
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.tfBuffer.lookup_transform(frame, enemy_closest_name, rospy.Time())
            trans = trans_stamped.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return False, 0, 0
        return True, trans.translation.x, trans.translation.y

    def getEnemyDistRad(self):
        try:
            # <class 'geometry_msgs.msg._TransformStamped.TransformStamped'>
            base_footprint_name=self.robot_namespace+'/base_footprint'
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.tfBuffer.lookup_transform(base_footprint_name, enemy_closest_name, rospy.Time())
            trans = trans_stamped.transform
            # trans = self.tfBuffer.lookup_transform('enemy_closest', "base_footprint", rospy.Time(), rospy.Duration(4))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rate.sleep()
            # rospy.logwarn(e)
            return False, 0, 0
        # rospy.loginfo(trans)
        # rospy.loginfo(type(trans))
        
        dist = (trans.translation.x**2 + trans.translation.y**2)**0.5
        rad = atan2(trans.translation.y, trans.translation.x)
        # print ("trans.translation.x:{}, trans.translation.y:{}".format(trans.translation.x, trans.translation.y))

        # rot = trans.rotation
        # rad = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

        return True, dist, rad
    
    def pubDetectRange(self, inner, outer):
        marker = Marker()
        marker.header.frame_id = self.robot_namespace+'/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.type = Marker.CYLINDER
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = inner * 2
        marker.scale.y = inner * 2
        marker.scale.z = 0.3
        marker.id = 0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.detect_inner_pub.publish(marker)

        marker.scale.x = outer * 2
        marker.scale.y = outer * 2
        marker.scale.z = 0.2
        marker.id = 1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.detect_outer_pub.publish(marker)



if __name__ == '__main__':
    rospy.init_node('my_run')
    bot = RandomBot('myRun')
    bot.strategy()
