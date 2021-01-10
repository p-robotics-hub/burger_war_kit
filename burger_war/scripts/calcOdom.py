#!/usr/bin/env python
# -*- coding: utf-8 -*-

# enemy.py
# write by hiroki ikeuchi @hotic06


import math
import rospy
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

class CalcOdom():
    # Static values
    TICK2RAD=1
    WHEEL_RADIUS=0.033
    odom_header_frame_id="odom"
    odom_child_frame_id="base_footprint"
    odom_topic_name="odom"


    def reset_pose(self,initial_pose):
        self.prev_update_time=rospy.get_time()
        self.last_theta=0.0
        self.odom_pose = initial_pose
        self.odom_vel=[0.0,0.0,0.0]
        self.last_velocity_left=0.0
        self.last_velocity_right=0.0
        self.last_tick_left = 0.0
        self.last_tick_right = 0.0

        # flag
        self.imu_ok=False
        self.jointstate_ok=False

    def __init__(self,initial_pose=[0,0,0]):        
        rospy.loginfo(initial_pose)

        self.initial_pose=initial_pose

        self.reset_pose(initial_pose)

        # publisher
        self.odom_pub = rospy.Publisher(self.odom_topic_name, Odometry,queue_size=1)

        # subscriber
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)
        self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)



        self.odom = Odometry()
        self.odom_tf = geometry_msgs.msg.TransformStamped()
        self.tf_broadcaster = tf.TransformBroadcaster()

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        self.imu_ok=True
        #rospy.loginfo(self.imu.orientation)

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        self.jointstate_ok=True
        #rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        #rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

    def updateMotorInfo(self,left_tick,right_tick):
        self.last_diff_tick_left = left_tick - self.last_tick_left
        self.last_diff_tick_right = right_tick - self.last_tick_right

        self.last_tick_left=left_tick
        self.last_tick_right=right_tick


    def calcOdometory(self,step_time,stamp):
        wheel_l = wheel_r = 0.0
        delta_s = delta_theta = theta = 0.0
        v = w = 0.0

        if step_time == 0:
            return False

        wheel_l = self.TICK2RAD * self.last_diff_tick_left
        wheel_r = self.TICK2RAD * self.last_diff_tick_right

        delta_s     = self.WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0
        

        orientation = [self.imu.orientation.w, self.imu.orientation.x , self.imu.orientation.y, self.imu.orientation.z]

        theta = math.atan2(orientation[1]*orientation[2] + orientation[0]*orientation[3],
                0.5  - orientation[2]*orientation[2] - orientation[3]*orientation[3])
        
        delta_theta = theta - self.last_theta

        if delta_theta > math.pi :
            delta_theta -= math.pi * 2
        elif delta_theta < -math.pi :
            delta_theta += math.pi * 2


        # compute odometric pose
        self.odom_pose[0] += delta_s * math.cos(self.odom_pose[2] + (delta_theta / 2.0))
        self.odom_pose[1] += delta_s * math.sin(self.odom_pose[2] + (delta_theta / 2.0))
        self.odom_pose[2] += delta_theta

        # compute odometric instantaneouse velocity

        v = delta_s / step_time
        w = delta_theta / step_time

        self.odom_vel[0] = v
        self.odom_vel[1] = 0.0
        self.odom_vel[2] = w

        #rospy.loginfo(self.odom_vel)

        self.last_velocity_left  = wheel_l / step_time
        self.last_velocity_right = wheel_r / step_time
        self.last_theta = theta

        self.odom.pose.pose.position.x = self.odom_pose[0]
        self.odom.pose.pose.position.y = self.odom_pose[1]
        self.odom.pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,self.odom_pose[2])
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]
        
        
        self.odom.twist.twist.linear.x = self.odom_vel[0]
        self.odom.twist.twist.angular.z = self.odom_vel[2]

        self.odom.header.stamp=stamp

        self.odom.header.frame_id=self.odom_header_frame_id
        self.odom.child_frame_id=self.odom_child_frame_id

        return self.odom

    def updateTF(self):
        self.odom_tf.header = self.odom.header
        self.odom_tf.child_frame_id = self.odom.child_frame_id
        self.odom_tf.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_tf.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_tf.transform.translation.z = self.odom.pose.pose.position.z
        self.odom_tf.transform.rotation = self.odom.pose.pose.orientation

    def loop(self):

        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.jointstate_ok and self.imu_ok:
                now=rospy.get_time()
                step_time = now - self.prev_update_time
                self.prev_update_time=now
                self.updateMotorInfo(self.wheel_rot_l,self.wheel_rot_r)
                self.calcOdometory(step_time,rospy.Time.now())

                self.odom_pub.publish(self.odom)
                #rospy.loginfo(self.odom)

                self.updateTF()

                self.tf_broadcaster.sendTransformMessage(self.odom_tf)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.reset_pose(self.initial_pose)

if __name__ == '__main__':

    rospy.sleep(2)
    
    rospy.init_node('calcodom')    

    initial_theta=rospy.get_param("~initial_theta",0.0)

    
    calc_odom = CalcOdom([0.0,0.0,initial_theta])

    rospy.sleep(2)
    calc_odom.loop()
