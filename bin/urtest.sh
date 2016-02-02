#!/bin/sh

# Extract robot URL from ROS_MASTER_URI environment varaible:
ROBOT_HOST=`echo $ROS_MASTER_URI | sed -e s,http://,, | sed -e s,:11311,,`
#echo ROBOT_HOST=$ROBOT_HOST

# This chunk of code probes the ROBOT_HOST to find out about its
# platform.  This is done copying short script over to
# ROBOT_HOST:/tmp/platform.sh, setting the execute bit for it, and
# using `ssh` to remotely execute it:
cat >/tmp/platform.sh<<EOF
#!/bin/bash
source /home/ubuntu/.profile
rosrun ubiquity_launches platform_probe.py
EOF
scp -q /tmp/platform.sh ubuntu@$ROBOT_HOST:/tmp
ssh ubuntu@$ROBOT_HOST chmod +x /tmp/platform.sh
PLATFORM=`ssh ubuntu@$ROBOT_HOST /tmp/platform.sh`
#echo PLATFORM=$PLATFORM

# Clear out file we use to indicate when the X11 channel is up:
rm -f /tmp/x11_up

# When we background the process, we want to be sure it gets killed 
# off when we type control-C.  The following two trap commands do this:
trap "exit" INT TERM
trap "kill 0" EXIT

# Now open the X11 channel using '&' to put it into the background.
# The last command will sleep for a million seconds to keep the channel
# open for a while:
(ssh -Y ubuntu@$ROBOT_HOST \
  'echo $DISPLAY > /tmp/display ; echo $DISPLAY; sleep 1000000' \
  1>&2) 2>/tmp/x11up &

# Now we wait for something to be written into `/tmp/x11up` before
# firing off the roslaunch:
while [ ! -s /tmp/x11up ] ;	\
    do sleep 1; 		\
    done

# Lastly, we need to create an executable file called /tmp/env_loader.sh
# over on ROBOT_HOST.  This file slurps in the ROS environment variables
# from `~/.profile` and sets the `DISPLAY` environment variable:
cat >/tmp/env_loader.sh<<EOF
#!/bin/bash
source ~/.profile
export DISPLAY=\`cat /tmp/display\`
exec "\$@"
EOF
scp -q /tmp/env_loader.sh ubuntu@$ROBOT_HOST:/tmp
ssh ubuntu@$ROBOT_HOST 'chmod +x /tmp/env_loader.sh'

# Now we can run the roslaunch command specifying both the PLATFORM
# and ROBOT_HOST:
echo roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST
roslaunch ubiquity_launches test.launch $PLATFORM robot_host:=$ROBOT_HOST

