#!/usr/bin/env python
# -*- coding: utf-8 -*-

#TF enemy position from ralative_pos topic
#Add time losed enemy to color_flag 

import rospy
import tf2_ros
import tf_conversions
import tf
import math
import roslib
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16MultiArray

class TransformEnemy():

    def __init__(self):
        #Get parameter
        #self.side = rospy.get_param("~side", "r")
        self.robot_namespace = rospy.get_param("~robot_namespace", "r")

        #Broadcaster, Subscriber
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.br_rel = tf2_ros.TransformBroadcaster()
        self.br_abs = tf2_ros.TransformBroadcaster()
        self.enemy_sub = rospy.Subscriber('relative_pose', PoseStamped, self.enemyCallback)
        self.flag_sub = rospy.Subscriber('color_flag', Int16MultiArray,self.flagCallback)
        self.flag_pub = rospy.Publisher('color_flag_time', Int16MultiArray, queue_size=1)

        #Initialize
        self.enemy_ps = PoseStamped()
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.position.x = 2.6
        self.enemy_ps.pose.orientation = rotation
        self.flags = [0, 0, 0, 0, 0, 0]
        self.initial_time = rospy.Time.now().secs

    def flagCallback(self, data):
        for i, flag in enumerate(data.data):
            self.flags[i] = flag
        if (self.flags[0] + self.flags[2] + self.flags[3]) == 0 :
            current_time = rospy.Time.now().secs
            self.flags[5] = current_time - self.initial_time
        self.publish_flags()
    
    def publish_flags(self):
        array_forPublish = Int16MultiArray(data=self.flags)
        self.flag_pub.publish(array_forPublish)

    def enemyCallback(self,data):
        self.enemy_ps.pose.position = data.pose.position
        self.enemy_ps.pose.orientation = data.pose.orientation
        if (self.flags[0] + self.flags[2] + self.flags[3]) != 0 :
            self.tf_rel_camera()
            self.tf_abs_camera()

    #Broadcast relative enmey from camera
    def tf_rel_camera(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_namespace+'/base_footprint'
        t.child_frame_id = 'enemy_camera_rel'
        t.transform.translation = self.enemy_ps.pose.position
        t.transform.rotation = self.enemy_ps.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    #Broadcast absolute enmey from camera
    def tf_abs_camera(self):
        try:
            map_topicname= self.robot_namespace + '/map'
            t = self.tfBuffer.lookup_transform(map_topicname, 'enemy_camera_rel', rospy.Time(0), rospy.Duration(1.0))
            t.header.frame_id = map_topicname
            t.child_frame_id = 'enemy_camera_abs'
            self.tf_broadcaster.sendTransform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def main(self):
        while not rospy.is_shutdown():
            rospy.spin()
        
if __name__ == '__main__':
    rospy.init_node('enemy_tf_broadcaster')
    br_enemy = TransformEnemy()
    br_enemy.main()
    

