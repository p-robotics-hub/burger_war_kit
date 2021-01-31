#!/bin/bash -x

cd $HOME/catkin_ws/src/burger_war_kit

BURGER_WAR_KIT_REPOSITORY=$HOME/catkin_ws/src/burger_war_kit
BURGER_WAR_AUTOTEST_LOG_REPOSITORY=$HOME/catkin_ws/src/burger_war_autotest
RESULTLOG=$BURGER_WAR_KIT_REPOSITORY/autotest/result.log
SRC_LOG=$RESULTLOG 
DST_LOG=$BURGER_WAR_AUTOTEST_LOG_REPOSITORY/result/result.log
LATEST_GITLOG_HASH="xxxx"

echo "iteration, enemy_level, game_time(s), date, my_score, enemy_score, battle_result, my_side" > $RESULTLOG

LOOP_TIMES=10

function do_game(){
    ITERATION=$1
    ENEMY_LEVEL=$2
    GAME_TIME=$3
    MY_SIDE=$4
    if [ -z $MY_SIDE ]; then
	MY_SIDE="r"
    fi

    # change directory
    pushd ${BURGER_WAR_KIT_REPOSITORY}

    # start
    gnome-terminal -- bash scripts/sim_with_judge.sh -s ${MY_SIDE}
    sleep 30
    gnome-terminal -- bash scripts/start.sh -l ${ENEMY_LEVEL} -s ${MY_SIDE}

    # wait game finish
    sleep $GAME_TIME

    # get result
    timeout 30s python autotest/get_score.py > out.log
    if [ $MY_SIDE == "r" ]; then
	MY_SCORE=`cat out.log | grep -w my_score | cut -d'=' -f2`
	ENEMY_SCORE=`cat out.log | grep -w enemy_score | cut -d'=' -f2`
    else
	# MY_SIDE != r, means mybot works enemy side..
	MY_SCORE=`cat out.log | grep -w enemy_score | cut -d'=' -f2`
	ENEMY_SCORE=`cat out.log | grep -w my_score | cut -d'=' -f2`
    fi
    DATE=`date --iso-8601=seconds`
    BATTLE_RESULT="LOSE"
    if [ $MY_SCORE -gt $ENEMY_SCORE ]; then
	BATTLE_RESULT="WIN"
    fi

    # output result
    echo "$ITERATION, $ENEMY_LEVEL, $GAME_TIME, $DATE, $MY_SCORE, $ENEMY_SCORE, $BATTLE_RESULT, $MY_SIDE" >> $RESULTLOG
    tail -1 $RESULTLOG
    
    # stop
    # wait stop until all process is end
    bash scripts/stop.sh -s true

    popd
}

<<COMMENTOUT
function check_latest_hash(){
    # check latest hash
    pushd $BURGER_WAR_KIT_REPOSITORY
    git pull
    GITLOG_HASH=`git log | head -1 | cut -d' ' -f2`
    if [ "$GITLOG_HASH" != "$LATEST_GITLOG_HASH" ];then
	echo "#--> latest commit:$GITLOG_HASH" >> $RESULTLOG
	LATEST_GITLOG_HASH=$GITLOG_HASH
    fi
    popd
}

function do_push(){

    # push
    pushd $BURGER_WAR_AUTOTEST_LOG_REPOSITORY/result
    git pull
    cp $SRC_LOG $DST_LOG
    git add $DST_LOG
    git commit -m "result.log update"
    git push
    popd
}
COMMENTOUT

# main loop
for ((i=0; i<${LOOP_TIMES}; i++));
do
    #check_latest_hash
    do_game ${i} 1 225 # 180 * 5/4 
    do_game ${i} 2 225 # 180 * 5/4 
    do_game ${i} 3 225 # 180 * 5/4
    #do_game ${i} 1 225 "b" # 180 * 5/4 # only enemy level1,2,3 works r side
    #do_game ${i} 2 225 "b" # 180 * 5/4 # 
    #do_game ${i} 3 225 "b" # 180 * 5/4 # 
#    do_game ${i} 4 225 # 180 * 5/4
#    do_game ${i} 5 225 # 180 * 5/4
#    do_game ${i} 6 225 # 180 * 5/4
#    do_game ${i} 7 225 # 180 * 5/4
#    do_game ${i} 8 225 # 180 * 5/4
    #do_push
done
