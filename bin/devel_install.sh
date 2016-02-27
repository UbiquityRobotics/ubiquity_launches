#!/usr/bin/env bash

set -x

# Install ROS packages:
update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
if [ -f /etc/apt/sources.list.d/ros-lastest.list ] ; then
    echo "/etc/apt/sources.list already points to ROS"
else
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
fi

# Install a bunch of tools:
sudo apt-get update
sudo apt-get install -y			\
  bmap-tools				\
  build-essential			\
  curl					\
  devscripts				\
  emacs					\
  equivs				\
  daemontools				\
  fakeroot				\
  gdebi-core				\
  git 					\
  joystick				\
  language-pack-en			\
  libnss-mdns				\
  mgetty				\
  minicom				\
  network-manager			\
  openssh-client			\
  openssh-server			\
  python				\
  python-bloom				\
  python-networkmanager			\
  python-pip				\
  python-rosdep				\
  python-serial				\
  ros-indigo-compressed-image-transport	\
  ros-indigo-desktop-full		\
  ros-indigo-joy			\
  ros-indigo-joystick-drivers		\
  ros-indigo-joystick-drivers		\
  ros-indigo-navigation			\
  ros-indigo-robot-model		\
  ros-indigo-ros-tutorials		\
  ros-indigo-serial			\
  ros-indigo-teleop-twist-joy		\
  ros-indigo-tf-conversions		\
  ros-indigo-tf2-geometry-msgs		\
  ros-indigo-turtlebot-teleop		\
  ros-indigo-xacro			\
  ros-indigo-yocs-velocity-smoother	\
  sshfs					\
  setserial				\
  ubuntu-standard			\
  vim					\
  wireless-tools			\
  wpasupplicant				\
  xterm					\
  zip

# Install the python debugger:
sudo pip install pudb