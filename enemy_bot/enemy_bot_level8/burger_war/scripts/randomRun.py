#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import rosparam
import random
import time

from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.vel_sub = rospy.Subscriber('cmd_vel',  Twist, self.update_timeem)
        self.stack_time = rospy.get_param("~stack", 5)
        self.time_em = rospy.Time.now().to_sec()
        self.time_now = rospy.Time.now().to_sec()
        self.flag = -3


    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 500:
            x = -0.2
            th = 0
        else:
            x = -0.2
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def update_timeem(self,msg):
        if not (msg.linear.x == 0.0 and msg.angular.z == 0.0):
            self.time_em = rospy.Time.now().to_sec()
        #print("Time now = ")
        #print(self.time_em)


    def strategy(self):
        r = rospy.Rate(0.5) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        while not rospy.is_shutdown():
            self.time_now = rospy.Time.now().to_sec()
            #print("Time now = ",)
            #print(self.time_now)
            self.time_dist = self.time_now - self.time_em 
            #print "Time distance = " ,self.time_dist
            if self.time_dist > self.stack_time:
                print ""
                print"EscapeRun"
                print ""
                for i in range(5):
                    twist = self.calcTwist()
                    #print(twist)
                    self.vel_pub.publish(twist)
                    rospy.sleep(0.1)     
            r.sleep()

def main():
        rospy.init_node('random_run')
        bot= RandomBot('Random')
        bot.strategy()
        


if __name__ == '__main__':
    rospy.sleep(20)
    main()

