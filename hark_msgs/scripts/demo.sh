#!/bin/sh

export BIN=/home/administrator/sensors_workspace/src/autonomy_hri/hark_msgs/scripts/hark
export DATA=${BIN}/MultiSpeech.wav
export DEVICE=plughw:1,0
export LOC=${BIN}/kinect_loc.dat
export SEP=${BIN}/kinect_sep.tff



if [ "$1" = "offline" ]
then
    echo "Offline mode"
       batchflow ${BIN}/demoOffline.n ${DATA} ${SEP} ${LOC}  ${BIN}/sep_files/offline 
elif [ "$1" = "online" ]
then
    echo "Online mode"
       batchflow ${BIN}/demoOnline_Ros.n ${DEVICE} ${SEP} ${LOC}  ${BIN}/sep_files/online 
    
else
    echo "usage: sh demo.sh [online|offline] [HRLE]"
    echo "   online : run from microphone"
    echo "   offline: run from wave file"
fi
