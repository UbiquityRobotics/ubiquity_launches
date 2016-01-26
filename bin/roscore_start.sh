#!/usr/bin/env bash

if [ -d "/opt/ros/indigo/bin" ] ; then
   source /opt/ros/indigo/setup.bash
fi
if [ -f ~/catkin_ws/devl/setup/sh ] ; then
    source ~/catkin_sw/devel/setup.bash
fi

export ROS_MASTER_URI=http://`hostname`.local:11311
export ROS_HOSTNAME=`hostname`.local

#echo PATH=$PATH
#env | grep ROS
roscore

