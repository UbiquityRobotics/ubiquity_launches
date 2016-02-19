cmake_minimum_required(VERSION 2.8.3)
project(ubiquity_launches)

find_package(catkin REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############
install(PROGRAMS
  bin/keyboard_drive
  bin/keyboard_navigate

  bin/loki_joystick_teleop
  bin/loki_local_costmap
  bin/loki_raspicam90
  bin/loki_rviz_local_costmap
  bin/loki_rviz_sonar

  bin/platform_probe
  bin/raspicam
  bin/raspicam_view
  bin/robot_base
  bin/roslauncher

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY m_fiducial_slam/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_fiducial_slam
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_joystick_teleop/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_joystick_teleop
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_keyboard_drive/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_keyboard_drive
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_keyboard_navigate/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_keyboard_navigate
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_move_base/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_move_base
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_move_base_view/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_move_base_view
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_raspicam_raw/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_raspicam_raw
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_raspicam_view/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_raspicam_view
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY m_robot_base/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/m_robot_base
  PATTERN ".svn" EXCLUDE)




install(DIRECTORY n_amcl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_amcl
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_bus_server/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_bus_server
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_camera/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_camera
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_cmd_vel_mux/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_cmd_vel_mux
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_fiducial_detect/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_fiducial_detect
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_fiducial_slam/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_fiducial_slam
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_image_uncompress/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_image_uncompress
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_joy/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_joy
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_keyboard_navigate/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_keyboard_navigate
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_map_server/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_map_server
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_motor_node/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_motor_node
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_move_base/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_move_base
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_raspicam/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_raspicam
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_relay/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_relay
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_robot_state_publisher/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_robot_state_publisher
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_rqt_image_view/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_rqt_image_view
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_rviz
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_sleep_forever/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_sleep_forever
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_spawner/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_spawner
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_teleop_twist_joy/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_teleop_twist_joy
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY n_teleop_twist_keyboard/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/n_teleop_twist_keyboard
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz_local_costmap/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_local_costmap
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz_sonar/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_sonar
  PATTERN ".svn" EXCLUDE)