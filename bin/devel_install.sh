#!/usr/bin/env bash

# Install ROS packages:
update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
if [ -f /etc/apt/sources.list.d/ros-lastest.list ] ; then
    echo "/etc/apt/sources.list already points to ROS"
else
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
fi
sudo apt-get update
sudo apt-get install -y ros-indigo-ros-desktop
