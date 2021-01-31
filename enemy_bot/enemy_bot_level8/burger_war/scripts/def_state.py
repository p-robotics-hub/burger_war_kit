#!/usr/bin/env python
# -*- coding: utf-8 -*-

class State(object):
    def __init__(self):
        self.states = ['search_enemy_distance', 'get_time_left', 'get_enemy_pose','get_nearest_target',
            'search_near_target','get_highest_target','go_to_target','escape']

        self.transitions = [
            {'trigger': 'enemy_far', 'source': 'search_enemy_distance', 'dest':'get_time_left'},
            {'trigger': 'enemy_near', 'source': 'search_enemy_distance', 'dest':'get_enemy_pose'},
            {'trigger': 'cannot_see_or_face', 'source': 'get_enemy_pose', 'dest':'get_time_left'},
            {'trigger': 'can_see_and_face', 'source': 'get_enemy_pose', 'dest':'escape'},
            {'trigger': 'time_over', 'source': 'get_time_left', 'dest':'get_nearest_target'},
            {'trigger': 'in_time', 'source': 'get_time_left', 'dest':'search_near_target'},
            {'trigger': 'near_target_exists', 'source': 'search_near_target', 'dest':'get_nearest_target'},
            {'trigger': 'near_target_absent', 'source': 'search_near_target', 'dest':'get_highest_target'},
            {'trigger': 'send_target', 'source': 'get_nearest_target', 'dest':'go_to_target'},
            {'trigger': 'send_target', 'source': 'get_highest_target', 'dest':'go_to_target'},
            {'trigger': 'cycle', 'source': 'go_to_target', 'dest':'search_enemy_distance'},
            {'trigger': 'cycle', 'source': 'escape', 'dest':'search_enemy_distance'}
        ]