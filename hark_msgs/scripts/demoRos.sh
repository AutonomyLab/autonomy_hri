#!/bin/sh

export BIN=$(rospack find hark_msgs)/scripts
export DATA=${BIN}/MultiSpeech.wav
export DEVICE=plughw:1,0
export CONF=${BIN}/kinect_tf.zip

if [ "$1" = "offline" ]
then
    echo "Offline mode"
    echo ${BIN}/demoOfflineRos.n ${DATA} ${CONF} Localization.txt \> log.txt
    ${BIN}/demoOfflineRos.n ${DATA} ${CONF} Localization.txt > log.txt
elif [ "$1" = "online" ]
then
    echo "Online mode"
    echo ${BIN}/demoOnlineRos.n ${DEVICE} ${CONF} Localization.txt \> log.txt
    ${BIN}/demoOnlineRos.n ${DEVICE} ${CONF} Localization.txt > log.txt
else
    echo "usage: sh demo.sh [online|offline]"
fi
