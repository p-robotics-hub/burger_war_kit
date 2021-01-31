#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Get enemy absolute position from TF and publish enemy position as absolute_pose
import roslib
import rospy
import tf2_ros
import tf_conversions
import tf
import math 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16MultiArray

class PubEnemyPose():

    def __init__(self):

        #Get parameter
        self.rate = rospy.get_param("~rate", 1)
        #self.side = rospy.get_param("~side", "r")

        #Transformer, Listener, Subscriber, Publisher
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.enemy_pub = rospy.Publisher('absolute_pos', PoseStamped, queue_size=10)
        self.flag_sub = rospy.Subscriber('/color_flag', Int16MultiArray,self.flagCallback)
        self.flag_pub = rospy.Publisher('color_flag_time', Int16MultiArray, queue_size=10)

        #Initialize 
        self.enemy_ps = PoseStamped()
        self.enemy_ps.pose.position.x = 1.3
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.orientation = rotation
        self.flags = [0, 0, 0, 0, 0, 0]
        self.initial_time = rospy.Time.now().secs


    def flagCallback(self, data):
        for i, flag in enumerate(data.data):
            self.flags[i] = flag
        if (self.flags[0] + self.flags[2] + self.flags[3]) == 0 :
            current_time = rospy.Time.now().secs
            self.flags[5] = current_time - self.initial_time

    def publish_flags(self):
        array_forPublish = Int16MultiArray(data=self.flags)
        self.flag_pub.publish(array_forPublish)

    def lis_pub_enemy_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        try:
            t = self.tfBuffer.lookup_transform('map', 'enemy_pos', rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            #rospy.logerr('LookupTransform Eroor !')
        if (self.flags[0] + self.flags[2] + self.flags[3]) != 0 :
            self.enemy_ps.pose.position = t.transform.translation
            self.enemy_ps.pose.orientation = t.transform.rotation
        msg.pose = self.enemy_ps.pose
        self.enemy_pub.publish(msg)

    def main(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            self.publish_flags()
            self.lis_pub_enemy_pose()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pub_enemy_abs')
    pub_enemy = PubEnemyPose()
    try:
        pub_enemy.main()

    except rospy.ROSInterruptException: pass