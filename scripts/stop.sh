#!/bin/bash

# option
IS_SYNC_KILL="false"
while getopts s: OPT
do
    case $OPT in
        "s" ) IS_SYNC_KILL="$OPTARG" ;;
    esac
done
echo "IS_SYNC_KILL: ${IS_SYNC_KILL}"

function try_kill_process_main(){
    PROCESS_NAME=$1
    if [ -z "$PROCESS_NAME" ]; then
	return 1
    fi
    
    PROCESS_ID=`ps -e -o pid,cmd | grep ${PROCESS_NAME} | grep -v grep | awk '{print $1}'`
    if [ -z "$PROCESS_ID" ]; then
	echo "no process like... ${PROCESS_NAME}"
	return 2
    fi
    echo "kill process ... ${PROCESS_NAME}"
    #kill -SIGINT $PROCESS_ID
    kill $PROCESS_ID

    return 0
}

function try_kill_process(){
    PROCESS_NAME=$1

    DURATION=2
    LOOP_TIMES=100
    for i in `seq ${LOOP_TIMES}`
    do
	try_kill_process_main ${PROCESS_NAME}
	RET=$?
	if [ $RET == 2 ];then
	    # if no process, break
	    break
	fi

	if [ $IS_SYNC_KILL != "true" ];then
	    # if not sync kill, break
	    break
	fi	
	sleep ${DURATION}
    done


}

function stop_process(){    
    try_kill_process "judgeServer.py"
    try_kill_process "JudgeWindow.py"
    try_kill_process "start.sh"
    try_kill_process "sim_with_judge.sh"    
    try_kill_process "setup_sim.launch"
    try_kill_process "recordmydesktop"
    try_kill_process "gazebo"
    try_kill_process "gzserver"
}

stop_process
