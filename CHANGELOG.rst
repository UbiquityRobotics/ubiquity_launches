^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ubiquity_launches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
