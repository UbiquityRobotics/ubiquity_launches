#!/usr/bin/env python

import rospy
import time

rospy.init_node("robot_loop")
while True:
    print("robot")
    time.sleep(1)
