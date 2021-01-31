#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Get enemy absolute position from TF and publish enemy position as absolute_pose
import roslib
import rospy
import tf2_ros
import tf_conversions
import tf
import math 
from geometry_msgs.msg import PoseStamped , Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg      import Int16MultiArray
from std_msgs.msg      import Bool 
import numpy as np

class PubEnemyPose():

    def __init__(self):

        #Get parameter
        self.rate = rospy.get_param("~rate", 5)
        self.side = rospy.get_param("~side", "r")
        self.robot_namespace = rospy.get_param("~robot_namespace", '')

        #Transformer, Listener, Subscriber, Publisher
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.enemy_pub = rospy.Publisher('absolute_pos', PoseStamped, queue_size=10)
        self.flag_sub = rospy.Subscriber('color_flag_time', Int16MultiArray,self.flagcolorCallback)
        self.flag_sub = rospy.Subscriber('lidar_flag', Bool, self.flaglidarCallback)

        #Initialize 
        self.enemy_ps = PoseStamped()
        self.enemy_ps.pose.position.x = 1.3
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.orientation = rotation
        self.flag_color = False
        self.flag_lidar = False
        self.t_camera = PoseStamped()
        self.t_lidar = PoseStamped()
        if self.side == "r":
            pose = Point(1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        else:
            pose = Point(-1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,0)
            orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        self.t_camera.pose.position = pose
        self.t_camera.pose.orientation = orientation
        self.t_camera.pose.position = pose
        self.t_lidar.pose.orientation = orientation

    def lisn_enemy_camera(self):
        try:
            map_topicname= self.robot_namespace + '/map'
            t = self.tfBuffer.lookup_transform(map_topicname, 'enemy_camera_abs', rospy.Time(0), rospy.Duration(1.0))
            self.t_camera.pose.position = t.transform.translation
            self.t_camera.pose.orientation = t.transform.rotation
            self.flag_color = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            #self.flag_color = False
    
    def lisn_enemy_lidar(self):
        try:
            map_topicname= self.robot_namespace + '/map'
            enemy_lidar_topicname= self.robot_namespace + '/enemy_lidar'
            t = self.tfBuffer.lookup_transform(map_topicname, enemy_lidar_topicname, rospy.Time(0), rospy.Duration(1.0))
            self.t_lidar.pose.position = t.transform.translation
            self.t_lidar.pose.orientation = t.transform.rotation
            self.flag_lidar = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            #self.flag_lidar = False

    def pub_enemy_abs(self):
        if self.flag_lidar == True:
            msg = PoseStamped()
            msg.header.frame_id = self.robot_namespace + '/map'
            msg.pose.position = self.t_lidar.pose.position
            msg.pose.orientation = self.t_lidar.pose.orientation
            self.enemy_pub.publish(msg)
            #rospy.loginfo("Publish Lidar pose")
        elif self.flag_color == True:
            msg = PoseStamped()
            msg.header.frame_id = self.robot_namespace + '/map'
            msg.pose.position = self.t_camera.pose.position
            msg.pose.orientation = self.t_camera.pose.orientation
            self.enemy_pub.publish(msg)
            #rospy.loginfo("Publish Camera pose")

    def flagcolorCallback(self,flag):
        if (flag.data[0] + flag.data[2] + flag.data[3]) == 0 :
            self.flag_color = False
        else:
            self.flag_color = True

    def flaglidarCallback(self,flag):
        self.flag_lidar = flag

    def main(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            self.lisn_enemy_camera()
            self.lisn_enemy_lidar()
            if (self.flag_color==True) or (self.flag_lidar==True):
                self.pub_enemy_abs()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pub_enemy')
    pub_enemy = PubEnemyPose()
    pub_enemy.main()
