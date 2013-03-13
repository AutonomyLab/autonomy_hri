#!/bin/bash
if [ -z "$1" ]
then
    echo "Please provide the folder name."
else

        mkdir $1
        cd $1
        rosbag record bluemax/human bluemax/output_rgb_debug /election gonk/human  gonk/output_rgb_debug fum/human fum/output_rgb_debug
    
fi
