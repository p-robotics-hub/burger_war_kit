from transitions.extensions import GraphMachine

states = ['search_enemy_distance', 'get_time_left', 'get_enemy_pose','get_nearest_target',
    'search_near_target','get_highest_target','go_to_target','escape']

transitions = [
    {'trigger': 'enemy_far', 'source': 'search_enemy_distance', 'dest':'get_time_left'},
    {'trigger': 'enemy_near', 'source': 'search_enemy_distance', 'dest':'get_enemy_pose'},
    {'trigger': 'cannnot_see_or_face', 'source': 'get_enemy_pose', 'dest':'get_time_left'},
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


class Matter(object):
    pass

model = Matter()
machine = GraphMachine(model=model, states=states, initial='search_enemy_distance', 
                       transitions=transitions,
                       auto_transitions=False, ordered_transitions=False,
                       title="", show_auto_transitions=False, show_conditions=False)

file_name = 'test.png'
model.get_graph().draw(file_name, prog = 'dot', format='png')

"""
while True:
    if model.state == 'search_enemy_distance':
        enemy_distance = get_enemy_distance()
        if enemy_distance > D:
            model.trigger('enemy_far')
        else:
            model.trigger('enemy_near')
    
    elif model.state == 'get_time_left':
        time_left = get_time_left()
        if time_left > T:
            model.trigger('in_time')
        else:
            model.trigger('time_over')
    
    elif model.state == 'get_enemy_pose':
        enemy_pose = get_enemy_pose()
        if enemy_pose and my_pose_orientation - enemy_pose_orientation < np.pi/4:
        else:
            model.trigger('cannot_see_or_face')
    
""" 