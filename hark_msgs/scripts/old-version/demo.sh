#!/bin/sh

export BIN=$(rospack find hark_msgs)/scripts/
export DATA=${BIN}/MultiSpeech.wav
export DEVICE=plughw:1,0
export CONF=${BIN}/kinect_tf.zip

if [ "$1" = "offline" ]
then
    echo "Offline mode"
    echo ${BIN}/demoOffline.n ${DATA} ${CONF} Localization.txt \> log.txt
         ${BIN}/demoOffline.n ${DATA} ${CONF} Localization.txt > log.txt
elif [ "$1" = "online" ]
then
    echo "Online mode"
    echo ${BIN}/demoOnline_Ros.n ${DEVICE} ${CONF} Localization.txt \> log.txt
         ${BIN}/demoOnline_Ros.n ${DEVICE} ${CONF} Localization.txt > log.txt
else
    echo "usage: sh demo.sh [online|offline]"
fi

