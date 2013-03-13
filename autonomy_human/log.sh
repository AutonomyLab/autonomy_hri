#!/bin/bash
if [ -z "$1" ]
then
    echo "Please provide the folder name."
else
    if [ -z "$2" ]
    then
        echo "Please provide namespace."
    else
        mkdir $2-$1
        cd $2-$1
        rosbag record $2/human $2/ardrone/image_raw $2/output_rgb_debug /election
    fi
fi
