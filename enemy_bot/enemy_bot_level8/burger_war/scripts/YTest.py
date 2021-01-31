#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random

from geometry_msgs.msg import Twist


class YBot():
    def __init__(self, bot_name="NoName",x = 0,th = 0):
        # bot name 
        self.name = bot_name
        # velocity argument
        self.vel_x = x 
        self.vel_th = th 
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)


    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        twist = Twist()
        twist.linear.x = self.vel_x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.vel_th

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        if not rospy.is_shutdown():
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


def main():
    rospy.init_node('Y_run')
    bot = YBot('Y_test')
    bot.strategy()


if __name__ == '__main__':
    main()