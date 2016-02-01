#!/bin/sh

# Extract robot URL from ROS_MASTER_URI environment varaible:
ROBOT_HOST=`echo $ROS_MASTER_URI | sed -e s,http://,, | sed -e s,:11311,,`
#echo ROBOT_HOST=$ROBOT_HOST

# Probe to find out what kind of platform the robot is:
PLATFORM=`ssh ubuntu@$ROBOT_HOST /home/ubuntu/catkin_ws/src/ubiquity_launches/bin/platform_probe.py`
#echo PLATFORM=$PLATFORM

# Clear out file we use to indicate when the X11 channel is up:
rm -f /tmp/x11_up

# When we background the process, we want to be sure it gets killed 
# off when we type control-C.  The following two trap commands do this:
trap "exit" INT TERM
trap "kill 0" EXIT

# Now open the X11 channel:
(ssh -Y ubuntu@$ROBOT_HOST \
  'echo export DISPLAY=$DISPLAY > /tmp/display.sh ; echo $DISPLAY; sleep 260' \
  1>&2) 2>/tmp/x11up &

# Now we wait for something to be written into `/tmp/x11up` before
# firing off the roslaunch:
while [ ! -s /tmp/x11up ] ;	\
    do sleep 1; 		\
    done

# Now we can run the roslaunch command specifying both the PLATFORM
# and ROBOT_HOST:
echo roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST
roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST

