#!/bin/bash

# install below
#   $ sudo apt-get install -y recordmydesktop gtk-recordmydesktop
#   $ sudo apt-get install -y ffmpeg

# default value
FPS="3"          # capture frame per sec
CAPTURE_NAME="capture.mp4"
MODE="start"

# get option
while getopts f:m:n: OPT
do
  case $OPT in
    "f" ) FPS="$OPTARG" ;;
    "m" ) MODE="$OPTARG" ;;
    "n" ) CAPTURE_NAME="$OPTARG" ;;
  esac
done


if [ $MODE == "start" ];then
    # capture start
    gnome-terminal -- recordmydesktop --no-sound --no-cursor --fps ${FPS}

elif [ $MODE == "stop" ];then
    #capture stop
    TMP_FNAME_OGV="out.ogv"
    TMP_FNAME_MP4="out.mp4"
    TMP2_FNAME_MP4="out2.mp4"    
    rm -f $TMP_FNAME_OGV
    rm -f $TMP_FNAME_MP4
    rm -f $TMP2_FNAME_MP4
    # kill process
    PROCESS_ID=`ps -e -o pid,cmd | grep "recordmydesktop" | grep -v grep | awk '{print $1}'`
    kill $PROCESS_ID
    sleep 30
    # save video
    ffmpeg -i $TMP_FNAME_OGV -vf scale=1024:-1 $TMP_FNAME_MP4
    ffmpeg -i $TMP_FNAME_MP4 -vf setpts=PTS/3.3 -af atempo=2.0 $TMP2_FNAME_MP4
    mv $TMP2_FNAME_MP4 ${CAPTURE_NAME}
    rm -f $TMP_FNAME_OGV
    rm -f $TMP_FNAME_MP4
    rm -f $TMP2_FNAME_MP4
else
    echo "do nothing"
fi
