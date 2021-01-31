#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import json
import sys

class Sample():

    def __init__(self):
        # war state
        topicname_war_state = "war_state"
        self.war_state = rospy.Subscriber(topicname_war_state, String, self.stateCallback)
        self.my_score = 0
        self.enemy_score = 0
        self.FLAG = 0
        self.COUNT = 0

    def stateCallback(self, state):

        dic = json.loads(state.data)

        if self.FLAG == 1:
            return 0

        self.my_score = int(dic["scores"]["r"])
        self.enemy_score = int(dic["scores"]["b"])

        print('my_score=%d' % self.my_score)
        print('enemy_score=%d' % self.enemy_score)
        
        self.FLAG = 1
        
    def strategy(self):
        # Main Loop --->
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()
            if self.FLAG > 0:
                sys.exit(0)

            # error exit
            # the case of "no state callback is published".
            # basically state callback come in 1s,
            # but sometimes no state callback. when it occurs, go exit..
            self.COUNT += 1
            if self.COUNT > 3000:
                sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('sample')
    bot = Sample()
    bot.strategy()

