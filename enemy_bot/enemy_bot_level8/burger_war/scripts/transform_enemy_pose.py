#!/usr/bin/env python
# -*- coding: utf-8 -*-

#TF enemy position from ralative_pos topic
#Add time losed enemy to color_flag 

import rospy
import tf2_ros
import tf_conversions
import tf
import math
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

class TransformEnemy():

    def __init__(self):
        #Get parameter
        self.rate = rospy.get_param("~rate", 1)
        #self.side = rospy.get_param("~side", "r")

        #Broadcaster, Subscriber
        self.br = tf2_ros.TransformBroadcaster()
        self.enemy_sub = rospy.Subscriber('relative_pose', PoseStamped, self.enemyCallback)
        
        #Initialize
        self.enemy_ps = PoseStamped()
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.position.x = 2.6
        self.enemy_ps.pose.orientation = rotation


    def enemyCallback(self,data):
        self.enemy_ps.pose.position = data.pose.position
        self.enemy_ps.pose.orientation = data.pose.orientation

    
    def tf_enemy_pose(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'enemy_pos'
        t.transform.translation = self.enemy_ps.pose.position
        t.transform.rotation = self.enemy_ps.pose.orientation
        self.br.sendTransform(t)
    

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.tf_enemy_pose()
            rate.sleep()

    
        
if __name__ == '__main__':
    rospy.init_node('enemy_tf_broadcaster')
    br_enemy = TransformEnemy()
    br_enemy.main()
    

