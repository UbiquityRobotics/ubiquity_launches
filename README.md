# loki_robot

This package provides a bunch of launch files for using the Loki robot.

## bringup

The `bringup` launch file is started as:

        roslaunch loki_robot bringup.launch

This just starts the ROS arduino node for now.


## description

The `description` launch files bring up the URD information.

On the robot:

        roslaunch loki_robot description.launch

will start the `ros_state_publisher` node with the `loki.urdf` file.
This will cause a bunch senor frames to be added to the base_link
frame in TF.

On the desktop/laptop:

        roslaunch loki_robot rviz_description.launch

will start rviz with TF visualiztion turned on.

## sonar

The `sonar` launch files bring up the Loki platform with the
sonars turned on.

On the robot:

        roslaunch loki_robot sonar.launch

get the platform started.

On the desktop/laptop:

        roslaunch loki_robot rviz_sonar.launch

will bring rviz with sonar sensor visualization turned on.

## local_costmap

The `local_costmap` launch files bring up Loki platform with
the sonars turned on *and* the rviz visualization is showing
the local costmap.

On the robot:

        roslaunch loki_robot local_costmap.launch

get the platform runnng with the sonars turned on.

On the desktop/laptop:

        roslaunch loki_robot rviz_local_costmap.launch

will bring up rviz with the local costmap visualiation turnd on.


