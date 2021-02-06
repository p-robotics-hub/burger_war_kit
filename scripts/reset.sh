#!/bin/bash

### 
#  reset gazebo, judgeserver.
#  
#  usage)
#  $ cd ~/catkin_ws/src/burger_war_kit
#  $ bash scripts/reset.sh
###

# reset judge
# check arg num
if [ $# -ne 2 ]; then
    RED_NAME="you"
    BLUE_NAME="enemy"
else
    RED_NAME=$1
    BLUE_NAME=$2
fi
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000  $RED_NAME $BLUE_NAME

# reset gazebo
rosservice call /gazebo/reset_simulation "{}"
