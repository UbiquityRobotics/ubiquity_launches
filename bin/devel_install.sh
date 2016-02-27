#!/usr/bin/env bash

# This script file sets up 64-bit x86 Ubuntu 14.04 machine to have all the
# stuff that Ubiquity Robotics wants installed on the machine.
#
# This script is reentrant that it can be run over and over again with out
# breaking anything.

# For script debugging:
set -x

# Verify that we are on a 64-bit platform:
MACHINE_ARCH = `uname -m`
if [ "$MACHINE_ARCH" != "x86_64" ] ; then
    echo "This machine script only works for 64-bit x86 (i.e. x86_64), not $MACHINE_ARCH"
    exit 1
fi

# Verify that we are on a Unbuntu:
DISTRO_ID = `lsb_release -s -i`
if [ "$DISTRO_ID" != "Ubuntu" ] ; then
    echo "ROS only runs on a Ubuntu distribution, not $DISRO_ID"
    exit 1
fi

# Verify that we are on 14.04:
DISTRO_VERSION = `lsb_release -s -r`
if [ "$DISTRO_VERSION" != "14.04" ] ; then
    echo "This script file only works for Ubuntu 14.04, not Ubuntu $DISTRO_VERSION"
    exit 1
fi

# Let the user know that we have sudo embedded in this script:
sudo echo "This shell script uses `sudo` to install software"

# Make sure that bash has its local information set:
update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Make the ROS indigo repository available to `apt-get`:
if [ -f /etc/apt/sources.list.d/ros-lastest.list ] ; then
    echo "/etc/apt/sources.list already points to ROS"
else
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
fi

# Update the apt-get database so it can find everything:
sudo apt-get update

# Install packages (alphabetical order):
sudo apt-get install -y                 \
  bmap-tools                            \
  build-essential                       \
  curl                                  \
  devscripts                            \
  emacs                                 \
  equivs                                \
  daemontools                           \
  fakeroot                              \
  fuse                                  \
  gdebi-core                            \
  git                                   \
  joystick                              \
  language-pack-en                      \
  libnss-mdns                           \
  mgetty                                \
  minicom                               \
  network-manager                       \
  openssh-client                        \
  openssh-server                        \
  python                                \
  python-bloom                          \
  python-networkmanager                 \
  python-pip                            \
  python-rosdep                         \
  python-serial                         \
  ros-indigo-compressed-image-transport \
  ros-indigo-desktop-full               \
  ros-indigo-joy                        \
  ros-indigo-joystick-drivers           \
  ros-indigo-joystick-drivers           \
  ros-indigo-navigation                 \
  ros-indigo-robot-model                \
  ros-indigo-ros-tutorials              \
  ros-indigo-serial                     \
  ros-indigo-teleop-twist-joy           \
  ros-indigo-tf-conversions             \
  ros-indigo-tf2-geometry-msgs          \
  ros-indigo-turtlebot-teleop           \
  ros-indigo-xacro                      \
  ros-indigo-yocs-velocity-smoother     \
  setserial                             \
  sshfs                                 \
  ubuntu-standard                       \
  vim                                   \
  wireless-tools                        \
  wpasupplicant                         \
  xterm                                 \
  zip

# Do we need:
#    wpa_suplicant
#    python-bloom

# Install the python debugger:
sudo pip install pudb

# Create the `/tmp/.ros_setup` script:
cat << EOF > /tmp/.ros_setup
# Unified ROS environment setup shell script.
#
# Only `source /opt/ros/indigo/setup.bash` if we have not already done so.
# We assume that this script does not change very often:
if [ -d "/opt/ros/indigo/bin" ] ; then
    case ":$PATH:" in
    *:/opt/ros/indigo/bin:*) ;;
    *) source /opt/ros/indigo/setup.bash ;;
    esac
fi

# Only `source ~/catkin_ws/devel/setup` if it exists:
if [ -f ~/catkin_ws/devel/setup.sh ] ; then
    source ~/catkin_ws/devel/setup.bash ;
fi

# Define some ROS environment variables:
export ROS_CATKIN_WS=~/catkin_ws
export ROS_HOSTNAME=`hostname`.local
export ROS_MASTER_URI=http://`hostname`.local:11311
export ROSLAUNCH_SSH_UNKNOWN=1
EOF

# Install `/tmp/.ros_setup` into `~/.ros_setup` if necessary:
if [ -f ~/.ros_setup ] ; then
    rm /tmp/.ros_setup
else
    mv /tmp/.ros_setup ~/.ros_setup
fi
    
# We need to make sure that we set up ROS for all instances of bash:
cat <<EOF >> /tmp/.bashrc_prefix
# We need to force the execution of `~/.ros_setup` for every instance
# of bash.  We do this by putting the line `source ~/.ros_setup` at
# the **BEGINNING** of ~/.bashrc.  The reason why it goes to the beginning
# is because when bash is not running in interactive mode, the rest of
# the contents of ~/.bashrc are simply not executed.
source ~/.ros_setup

EOF
# Check to see if we have already install the script:
if grep "~/.ros_setup" ~/.bashrc ; then
    # Already installed, so just clean-up /tmp:
    rm /tmp/.bashrc_prefix
else
    # Not installed, so build a new one in 
    cat /tmp/.bashrc_prefix ~/.basrc > /tmp/.bashrc
    mv /tmp/.bashc ~/.bashrc
fi

# Make sure that secure shell is setup:
if [ ! -f ~/.ssh/id_rsa ] ; then
    # We need to create private/public key for secure shell:
    echo "Creating public/private key pair for secure shell (this may take a while)"
    ssh-keygen -t rsa < /dev/null
fi

# Now make sure that we can securely login ourself:
echo "We need to be able to ssh to ourselves without a password prompt"
echo "Please type in your password below to make this happen"
ssh-copy-id `whoami`@`hostname`.local

# Now build our catkin workspace:
if [ ! -f catkin_ws/src ] ; then
    mkdir -p catkin_ws/src
    source ~/.ros_setup
    (cd catkin_ws; catkin_make)
fi


