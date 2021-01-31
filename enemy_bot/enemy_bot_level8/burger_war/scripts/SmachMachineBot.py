#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random


class search_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemy','flag'],input_keys = ['search_input'],output_keys = ['search_output'])

    def execute(self,userdata):
        #search progarm enemy  or flag
        rospy.loginfo('Executing state search_mode')
        value = random.randint(1,1000)
        print(value)
        rospy.sleep(4)
        if value < 500:
            userdata.search_output = value
            return 'enemy'
        else:
            userdata.search_output = value
            return 'flag'

class go_enemy(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'],
        input_keys = ['go_enemy_input'],
        output_keys = ['go_enemy_output'])

    def execute(self,userdata):
        rospy.loginfo('Executing state go_enemy')
        for index in range(userdata.go_enemy_input):
            #rospy.sleep(0.1)
            print('gogo')

        return 'success'

    
class go_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['enemy','search_mode'],
        input_keys = ['go_flag_input'],
        output_keys = ['go_flag_output'])

    def execute(self,userdata):
        rospy.loginfo('Executing state go_flag')
        value = random.randint(1,1000)
        print(value)
        for index in range(userdata.go_flag_input):
            print('gogo')
            #rospy.sleep(0.1)
        if value < 500 :
            return 'enemy'
        else:
            return 'search_mode'

    


def main():
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes = ['success'],
    input_keys = ['sm_input'],
    output_keys = ['sm_output'])
    sm_input = 0
    sm_output 

    with sm:
        # Add states to the container
        smach.StateMachine.add('search_mode', search_mode(), 
                               transitions = {'enemy':'enemy', 'flag':'flag'},
                                remapping = {'search_input':'sm_input',
                                             'search_output':'sm_data'})
        smach.StateMachine.add('enemy', go_enemy(), 
                               transitions = {'success':'success'},
                               remapping = {'go_enemy_input':'sm_data',
                                            'go_enemy_output':'sm_output'})
        smach.StateMachine.add('flag',go_flag(), 

                               transitions={'enemy':'enemy','search_mode':'search_mode'},
                               remapping = {'go_flag_input':'sm_data',
                                            'go_flag_output':'sm_data'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()