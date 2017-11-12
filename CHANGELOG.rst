^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ubiquity_launches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2016-02-18)
------------------
* Restore motor_node which was deleted
* Contributors: Rohan Agrawal

0.2.0 (2016-02-18)
------------------
* Update Cmakelists
* Restored n_move_base which I accidentally removed.
* Added travis build status to README
* use bash || for testing both ssh commands
* Hack in automatic user detection in roslauncher
* Replace all the old loki_camera stuff with raspicam and raspicam_view
* Got raspicam_view to work with new machine tag args
* Got m_raspicam_raw to work with new machine infasrtuture
* Replaced loki_base and magni_base with unified robot_base script
* Update m_move_base to reflect move of cmd_vel_mux
* Add all required things to sim
* Change default machine for n_relay to robot
* Machine tag in n_relay
* Moved machine def to the top of m_sim_base
* Move turtlebot_model to m_sim_base
* Using robot_base instead of stage_ros directly
* Support sim in robot_base
* Use not shutdown loop instead of infinite loop
* launching cmd_vel_mux with include
* Made cmd_vel_mux launcher (uses nodelets)
* Passing machine_name from m_move_base
* Machine name argumentizeation for rsp
* removed unnessary machine definition
* Further adding of machine_name args
* Added viewer args to more stuff
* Argument-zied machine name with reasonable defaults
* Pass robot host/user to amcl and map_server instead of viewer
* n_amcl and n_map_server now use machine_host and user
* Passing viewer/robot on keyboard_nav and move_base_view
* Have m_move_base require all nessacary args
* Removed old refrences to keyboard_drive
* n_rviz run through ssh
* Launch keyboard_naviagte though ssh to viewer
* Merged branch master into master
* Using ssh to localhost to run teleop
* Use generic machine args for loki_base nodes
* Add teleop_twist_keyboard with any machine format
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Banged together bin/yaml2launch.py
* Using move_base_view as the starter
* Luanch -> Launch
* Move viz to move_base_view
* Launching cmd_vel_mux on remote
* Fixed Typo, moved back to whoami
* New line for clarity of nodelet stuff
* More renaming to keyboard_navigate
* Output screen on teleop to make sure that we can see the output
* fix missing end bracket
* Switch robot_name to correct robot_host
* Robot_base launch is not xml
* fixed bug of mb vs rb
* Remo.ved an. exce.ess. period. in. fil.na.me
* Using ubuntu as user instead of whoami
* Change keyboard_drive to launch.xml
* Added keyboard_drive to unified-ily teleop
* Removed buggy space
* Rename folder for keyboard drive as well
* rename keyboard_drive to keyboard_navigate
* We ha`d an acc`ess of ac`cent ma`rks, it has been rectified
* Fixed always on roscore documentation
* Update Cmakelists to not fail on build
* Make sure robot_platform is being sent to loki_base correctly
* Using robot_base to auto switch between loki and magni
* Consolidate loki and magni base into robot base
* Get rid of loki_serial_master
* More tweaking of m_move_base.launch.xml
* More conversion of move_base.
* Starting to de-turtlebot m_move_base.
* Fixed raspicam_view
* Convert robot_base to robot_platform
* More launch file work.
* Fixed some typos
* Added recipe to configure Ubiquity Robotics development environment.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Remove old ros_launcher.py program
* Added n_sleep_forever to bring up roscore using robot_upstart package.
* More unified launch file development
* More work on unified launch files.
* Renamved platform_probe.py to platform_probe.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Changed platform to robot_base
* Some more work on unified launch files.
* Updated test.launch and urtest.sh
* Fixed a typo.
* Tweaked up bin/test.launch and bin/urtest.sh to work with one another.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Added platform_probe.py and ros_launche.pry to ubiquity_launches/bin
* Fixed up robot.py
* Added platform_probe.py and roscore_start.sh to ubiquity_launche/bin
* Contributors: Rohan Agrawal, Wayne C. Gramlich, Wayne Gramlich

* Added travis build status to README
* use bash || for testing both ssh commands
* Hack in automatic user detection in roslauncher
* Replace all the old loki_camera stuff with raspicam and raspicam_view
* Got raspicam_view to work with new machine tag args
* Got m_raspicam_raw to work with new machine infasrtuture
* Replaced loki_base and magni_base with unified robot_base script
* Update m_move_base to reflect move of cmd_vel_mux
* Add all required things to sim
* Change default machine for n_relay to robot
* Machine tag in n_relay
* Moved machine def to the top of m_sim_base
* Move turtlebot_model to m_sim_base
* Using robot_base instead of stage_ros directly
* Support sim in robot_base
* Use not shutdown loop instead of infinite loop
* launching cmd_vel_mux with include
* Made cmd_vel_mux launcher (uses nodelets)
* Passing machine_name from m_move_base
* Machine name argumentizeation for rsp
* removed unnessary machine definition
* Further adding of machine_name args
* Added viewer args to more stuff
* Argument-zied machine name with reasonable defaults
* Pass robot host/user to amcl and map_server instead of viewer
* n_amcl and n_map_server now use machine_host and user
* Passing viewer/robot on keyboard_nav and move_base_view
* Have m_move_base require all nessacary args
* Removed old refrences to keyboard_drive
* n_rviz run through ssh
* Launch keyboard_naviagte though ssh to viewer
* Merged branch master into master
* Using ssh to localhost to run teleop
* Use generic machine args for loki_base nodes
* Add teleop_twist_keyboard with any machine format
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Banged together bin/yaml2launch.py
* Using move_base_view as the starter
* Luanch -> Launch
* Move viz to move_base_view
* Launching cmd_vel_mux on remote
* Fixed Typo, moved back to whoami
* New line for clarity of nodelet stuff
* More renaming to keyboard_navigate
* Output screen on teleop to make sure that we can see the output
* fix missing end bracket
* Switch robot_name to correct robot_host
* Robot_base launch is not xml
* fixed bug of mb vs rb
* Remo.ved an. exce.ess. period. in. fil.na.me
* Using ubuntu as user instead of whoami
* Change keyboard_drive to launch.xml
* Added keyboard_drive to unified-ily teleop
* Removed buggy space
* Rename folder for keyboard drive as well
* rename keyboard_drive to keyboard_navigate
* We ha`d an acc`ess of ac`cent ma`rks, it has been rectified
* Fixed always on roscore documentation
* Update Cmakelists to not fail on build
* Make sure robot_platform is being sent to loki_base correctly
* Using robot_base to auto switch between loki and magni
* Consolidate loki and magni base into robot base
* Get rid of loki_serial_master
* More tweaking of m_move_base.launch.xml
* More conversion of move_base.
* Starting to de-turtlebot m_move_base.
* Fixed raspicam_view
* Convert robot_base to robot_platform
* More launch file work.
* Fixed some typos
* Added recipe to configure Ubiquity Robotics development environment.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Remove old ros_launcher.py program
* Added n_sleep_forever to bring up roscore using robot_upstart package.
* More unified launch file development
* More work on unified launch files.
* Renamved platform_probe.py to platform_probe.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Changed platform to robot_base
* Some more work on unified launch files.
* Updated test.launch and urtest.sh
* Fixed a typo.
* Tweaked up bin/test.launch and bin/urtest.sh to work with one another.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Added platform_probe.py and ros_launche.pry to ubiquity_launches/bin
* Fixed up robot.py
* Added platform_probe.py and roscore_start.sh to ubiquity_launche/bin
* Contributors: Rohan Agrawal, Wayne C. Gramlich, Wayne Gramlich

0.1.1 (2016-01-24)
------------------
* Added robot.py and test.launch .
* Merge branch 'master' of github.com:UbiquityRobotics/ubiquity_launches
* modifications to make loki_base work
* Updated README.md
* Updated README.md
* Added install rule for loki_base exec
* Add install rule for loki_base
  We need to find a better way to manage the install rules
* Added a loki_base exec file
* Added basic loki_base launch file
* Added Travis Button
* Dummy Commit for travis
* Added travis file
  added travis file to make sure cmake is valid
* Added install rules
  Added install rules for bin and all the launches to make sure files are avalible in non-devel environments, such as when installing from debs.
* Remove footprint_layer, which has been removed from ROS
* Correct default fidicual size
* Fix bugs with slam
* Fix bugs with camera
* More documentation.
* Updated README.md
* Sweep through and updated program comments, fix typos, etc.
* Added multiple base testing.
* Search for unused files.
* Fixed launch files for Magni base.
* Added some magni bring up launch files.  Not debugged though.
* Wrapped up initial file scraper.
* Wrote generate_launches_md.py file to scrape all the launch directories for documenation.
* Reworked the raspicam launch files to generate an uncompress image stream.
* Added more arguments.
* Update loki.yaml.
* Increased maximum speed for Loki to .25M/sec.
* Added raspicam stuff.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Add loki_raspicam and loki_raspicam90
* Fixed broken link again.
* Renamed ubiquity-misc to ubiquity_main.
* Added documentation for loki_camera and loki_view_camera.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Added loki_camera to bin.
* Merge branch 'master' of https://github.com/UbiquityRobotics/ubiquity_launches
* Added missing file n_map_server.launch.xml
* Fixed n_map_server
* Fixed an incorrect path to global map.
* Updated README.md.  Remove n_ros_arduino_bridge and rviz_loki_sonar directories.
* Finished cleaing up loki_robot except for documentation.
* Fixed up the loki_rvix executables.
* Removed loki_rviz_local_costmap
* Converted to rvzi_local_costmap.
* Switched over to new robot_base argument in .launch files.
* More work on rviz .launch files.
* Started cleaning out old stuff.  Added loki_rviz_local_costmap.
* Create loki_local_costmap .
* Continued work on local costmap launch file.
* More clean up of local cost map.
* More reworking of the local cost map .launch files.
* Switched over to m_joystick_teleop .launch file.
* Started using more <arg ...>'s.
* Converted remaining .launch files to use ubiquity_launches as the package name.
* Added n_bus_server node.
* Added rviz_loki_sonar .
* Updated README.md
* Started switching over to launch repository architecture.
* Documented proposed new structure for robot lauch repositories.
* Added xloki_joy
* Merge branch 'master' of https://github.com/UbiquityRobotics/loki_robot
* Tweaked some command for local_costmap.
* Set up the rviz_local_costmap.launch file and updated README.md .
* Moved fiducial_slam launch file to loki_robot repository.
* Moved fiducial_detect launch file to loki_robot repository.
* Moved camara launch file and calibration file to loki_robot repository.
* Moved joystick over to loki_robot repository.
* Moved map server to run out of loki_robot repository.
* Moved move_base parameters over to xlocal_local_costmap/params
* Got wayne.yaml into the repository.
* Added camera_pose to loki.urdf file.
* More work on local_costmap.launch
* More hacking on local_costmap.launch.
* Switched over to loki_robot robot state publisher
* Added new local_costmap.launch file.  This one still has hooks into the robot-configurtions repository (which is scheduled to go away.)
* Got odometry to work with bus_server.py.
* Merge branch 'master' of https://github.com/UbiquityRobotics/loki_robot
  Merge in new PID parameters for Loki
* Added exprimental/wayne
* Change PID parameters for new firmware changes
* Added experimental sub-directory to loki_robot repository.
* Added rviz_sonar.launch
* Merge branch 'master' of https://github.com/UbiquityRobotics/loki_robot
* Added sonar.launch .
* Added rviz_description.launch
* Missed description.launch
* Got loki_description sub-directory to work.
* Renamed ros_arduino_bridge.launch to bringup.launch.
* Rearranched files into task based sub-directories.
* Added some more launch files.
* Added package.xml
* Initial launch and configuration files.
* Initial commit
* Contributors: Mark Johnston, Rohan Agrawal, Tony Pratkanis, Wayne C. Gramlich, Wayne Gramlich, waynegramlich
