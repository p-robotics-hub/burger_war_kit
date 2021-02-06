#!/bin/bash

# To use recordmydesktop, install as below
#   $ sudo apt-get install recordmydesktop gtk-recordmydesktop
# in detail, see "man recordmydesktop"

# default value
SLEEP_TIME="225" # sec
FPS="1"          # capture frame per sec
CAPTURE_NAME="capture.ogv"

# get option
while getopts t:f:n: OPT
do
  case $OPT in
    "t" ) SLEEP_TIME="$OPTARG" ;;
    "f" ) FPS="$OPTARG" ;;
    "n" ) CAPTURE_NAME="$OPTARG" ;;    
  esac
done

# capture and sleep
PROCESS_NAME="recordmydesktop"
rm -f out.ogv
gnome-terminal -- recordmydesktop --no-sound --no-cursor --fps ${FPS}
sleep ${SLEEP_TIME}
PROCESS_ID=`ps -e -o pid,cmd | grep ${PROCESS_NAME} | grep -v grep | awk '{print $1}'`
kill $PROCESS_ID
sleep 30
mv out.ogv ${CAPTURE_NAME}
