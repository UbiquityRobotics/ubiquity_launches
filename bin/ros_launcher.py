#!/usr/bin/env python

import os
import subprocess
import xmlrpclib
import signal
import sys
import time

processes = {}

def shut_down():
    print("Shutting down all processes")
    for process_name in processes.keys():
	process = processes[process_name]
	print("terminate process {0}".format(process_name))
	process.terminate()
    time.sleep(5)
    for process_name in processes.keys():
	process = processes[process_name]
	print("kill process {0}".format(process_name))
	process.kill()
    for process_name in processes.keys():
	process = processes[process_name]
	print("wait for process {0}".format(process_name))
	process.wait()
	del processes[process_name]
    print("All processes should be dead")

def is_ros_core_up():
    ros_core_up = False
    caller_id = "/script"
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
	code, msg, val = m.getSystemState(caller_id)
	if code == 1:
	    ros_core_up = True
    except:
	pass
    return ros_core_up

def is_ssh_config_ok():
    # Open *ssh_config_file*:
    ssh_config_file_name = "/etc/ssh/ssh_config"
    ssh_config_file = open(ssh_config_file_name, "ra")
    if not isinstance(ssh_config_file, file):
	print("You must have a '{0}' file.  Try `sudo apt-get install ssh`".
	  format(ssh_config_file))
	return False

    # Read the rest of *sch_config_file* in as *lines*:
    lines = ssh_config_file.readlines()
    ssh_config_file.close()
	
    # Now make sure that whe have "ForwardX11" set to "yes" and
    # "ForwardX11Trusted" set to "yes".
    forward_x11 = False
    forward_x11_trusted = False
    for line_index in range(len(lines)):
	line = lines[line_index].strip()
	#print("line='{0}'".format(line))
	if "ForwardX11Trusted" in line:
	    #print("@@@@@@@@@@@@@@@@@@@@@@@@")
	    if "#" in line:
		print(("File '{0}' line{0}:" +
		  " You need to uncomment 'ForwardX11Trusted'").
		  format(ssh_config_file_name, line_index + 1))
		return False
	    elif "yes" in line:
		forward_x11_trusted = True
	    else:
		print("File '{0}' line{0}: Set 'ForwardX11Trusted' to 'yes'".
		  format(ssh_config_file_name, line_index + 1))
		return False
	elif "ForwardX11" in line:
	    #print("#########################")
	    if "#" in line:
		print("File '{0}' line{0}: You need to uncomment 'ForwardX11'".
		  format(ssh_config_file_name, line_index + 1))
		return False
	    elif "yes" in line:
		forward_x11 = True
	    else:
		print("File '{0}' line{0}: Set 'ForwardX11' to 'yes'".
		  format(ssh_config_file_name, line_index + 1))
		return False

    # Make sure that both *forward_x11* and *forward_x11_trusted* are set:
    if not forward_x11:
	print("File '{0}': 'ForwardX11 yes' is not in file".
	  format(ssh_config_file_name))
	return False
    if not forward_x11_trusted:
	print("File '{0}': 'ForwardX11Trusted yes' is not file".
	  format(ssh_config_file_name))
	return False

    return True

def main():
    signal.signal(signal.SIGINT, shut_down)
    try:
	main_helper()
    except:
	pass
    shut_down()
    return 0

def main_helper():
    environment_variables = os.environ

    # Lookup *ros_hostname* environment variable:
    if not "ROS_HOSTNAME" in environment_variables:
	print("ROS_HOSTNAME environment variable is not set")
	return 1
    ros_hostname = environment_variables["ROS_HOSTNAME"]

    # Verify that *ros_hostname* ends with ".local":
    if not ros_hostname.endswith(".local"):
	print("ROS_HOSTNAME = '{0}' which needs to have a '.local' suffix".
	  format(ros_hostname))
	return 1
    print("ros_hostname='{0}'".format(ros_hostname))
    local_host_name = ros_hostname

    # Lookup *ros_master_uri* environment variable:
    if not "ROS_MASTER_URI" in environment_variables:
	print("ROS_MASTER_URI environment variable is not set")
	return 1
    ros_master_uri = environment_variables["ROS_MASTER_URI"]

    # Grab *remote_host_name* from *ros_master_uri*:
    if not ros_master_uri.startswith("http://"):
	print("ROS_MASTER_URI does not start with 'http://'")
	return 1
    if not ros_master_uri.endswith(".local:11311"):
	print("ROS_MASTER_URI does not end with '.local:11311'")
	return 1
    remote_host_name = ros_master_uri[7:-6]
    print("remote_hostname='{0}'".format(remote_host_name))

    # Verify that /etc/ssh/ssh.conf

    if local_host_name == remote_host_name:
	# We are running the simulator:
	pass
    else:
	# We are running a remote robot:
	if not is_ssh_config_ok():
	    print("File '/etc/ssh/ssh_config' is not properly configured")
	    return 1

	# Verify that 'ROSLAUNCH_SSH_UNKNOWN' is set to 1:
	ros_launch_ssh_unknown = False
	try:
	    ros_launch_ssh_unknown = \
	      bool(int(environment_variables["ROSLAUNCH_SSH_UNKNOWN"]))
        except:
	    pass
	if not ros_launch_ssh_unknown:
	    print("'ROSLAUNCH_SSH_UNKNOWN' environment variable not set to 1")
	    return 1

	remote_login ="ubuntu@{0}".format(remote_host_name)

	# Figure what kind of remote platform the robot is:
	probe_results = subprocess.check_output(
	  ["ssh",
	    remote_login,
	    "~/catkin_ws/src/ubiquity_launches/bin/platform_probe.py"
	  ]).strip()
	print("probe_results='{0}'".format(probe_results))

	# Make sure roscore is up on the remote robot:
	ros_core_up = is_ros_core_up()
	if not ros_core_up:
	    roscore_process = subprocess.Popen(
	      ["ssh",
	       remote_login,
	       "~/catkin_ws/src/ubiquity_launches/bin/roscore_start.sh"
	      ])
	    processes["roscore"] = roscore_process
	    for count in range(15):
		time.sleep(1)
		if is_ros_core_up():
		    ros_core_up = True
		    break
	if not ros_core_up:
	    print("Unable to bring up `roscore`")
	    return 1
	print("roscore is up")

	# Open up an `ssh -X` channel and record the resulting DISPLAY
	# environment variable into `/tmp/display.sh`:
	print("Starting test.launch file")
	display_process = subprocess.Popen(
	  ["ssh",
	   "-X",
	   remote_login,
	   "echo \"export DISPLAY=$DISPLAY\" > /tmp/display.sh; sleep 1000000"
	  ])
	processes["display"] = display_process

	# Now fire off the roslaunch:
	subprocess.check_output(
	 ["roslaunch",
	  "ubiquity_launches",
	  "test.launch",
	 ])

	time.sleep(5)
	return 0
           
if __name__ == "__main__":
    main()
