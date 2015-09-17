# ubiquity_launches.

This repository contains a ROS launch and configuration files
for the Ubiquity Robots platforms.

It adheres to the
[Robot Launch Repository Standard](https://github.com/UbiquityRobotics/ubiquity-misc/blob/master/robot_launch_repositories.md) document.

This repository is broken into some categories:

* `bin`: The `bin` directory contains a bunch of executables shell
  scripts that fire off a launch file.

* `n_*`: The `n_*` directories contain the launch files and
  configuration files needed to launch a single ROS Node.

* `m_*`: The `m_*` directories will launch Multiple ROS nodes.

* `rviz_*: The `rviz_*` directories are used to launch the RViz
  program configured to view a corresponding robot program.

* others: Other directories contain miscellaneous launch files.

## The `bin` Directory

The executables in the `bin` directory are listed alphabetically
below:

* `loki_joystick_teleop`:
  This program is run on the robot and starts up a Loki platform
  with a PS3/XBox game controller to control robot motion.

* `loki_local_costmap`:
  This program is run on the robot and starts up a Loki platform
  that starts up robot that is running the both the PS3/XBox
  joystick nodes and the fiducial detection and slam nodes.
  The file is focused on generating a local cost map for viewing
  using the `loki_rviz_local_costmap` program.

* `loki_rviz_local_costmap`:
  This program is run on the laptop/desktop and brings up RViz
  in a mode that shows the robot, sonar sensors and local cost
  map.

* `loki_rviz_sonar`:
  This program is run on the laptop/desktop and brings up RViz
  in a mode that shows the robot sonar sensors.

## `m_*` Directories

The following `m_*` directories are listed in alphabetical
order below:

* `m_fiducal_slam`:
  The launch file for this directory fires off the fiducial
  slam subsystem.  This causes the `move_base` node to be
  loaded so that local cost maps can be generated.
  This launch file requires a `robot_base` argument to specify
  which robot base is being used (e.g `loki`, `magni`, etc.)

* `m_joystick_teleop`:
  The launch file for this directory fires off the joystick
  nodes to support the wireless PS2/XBox game controller for
  driving the robot around.
  This launch file requires a `robot_base` argument to specify
  which robot base is being used (e.g `loki`, `magni`, etc.)

## `n_*` Directories

The following `n_*` directories are used to fire up a single
ROS node.  They are listed alphabetically below:

* `n_bus_server`:
  The launch file for this directory starts the Ubiquity Robots
  [`bus_server`](https://github.com/UbiquityRobotics/bus_server)
  package that interfaces to the robot serial port.
  This launch file has the following arguments:
  * `robot_base`: (Required)

* `n_camera`:
  The launch file for this directory starts the camera node
  using the ROS [`gscam`](http://wiki.ros.org/gscam) package.
  This launch file has the following arguments:
  * `robot_base`: (Required)

* `n_fiducial_detect`:
  The launch file for this directory starts the fiducial detection
  portion of the Ubiquty Robots
  [fiducial](https://github.com/UbiquityRobotics/fiducials)
  that is used to detect ceiling fiducials for localization.
  This launch file has the following arguments:
  * `camera_frame`: (Default: `/camera/`)
  * `image`: (Default: `image_rect`)
  * `fiducial_len`: (Default: `0.146`)
  * `undistort_points`: (Default: `false`)

* `n_fiducial_slam`:
  The launch file for this directory starts the fiducial SLAM
  (Simultaneous Localization and Mapping) ROS node.
  This launch file has the following arguments:
  * `mapping_node`: (Default `false`)
  * `camera_frame`: (Required)
  * `map_frame`: (Required)
  * `odom_frame`: (Required)
  * `pose_frame`: (Required)

* `n_joy`:
  The launch file for this directory starts the ROS
  [joy](http://wiki.ros.org/joy) node.

* `n_loki_serial_master`:
  The launch file for this directory starts the `n_bus_server`
  node configured for the Loki platform.

* `n_map_server`:
  The launch file for this directory starts the ROS
  [`map_server`](http://wiki.ros.org/map_server) node.

* `n_move_base`:
  The launch file for this directory starts the ROS
  [`move_base`](http://wiki.ros.org/move_base) node.

* `n_robot_state_publisher`:
  The launch file for this directory starts the ROS
  [`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher) node.
  This launch file has the following arguments:
  * `robot_base`: (Required)

* `n_teleop_twist_joy`:
  The launch file for this directory starts the ROS
  [`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) node.
  This launch file has the following arguments:
  * `robot_base`: (Required)

## `rviz_*` Directories

* `rviz_local_costmap`:
  The launch file for this directory starts the ROS RViz
  in a mode that shows a local costmap.
  This launch file has the following arguments:
  * `robot_base`: (Required)

* `rviz_sonar`
  The launch file for this directory starts the ROS RViz
  in a mode that shows the sonar sensors.
  This launch file has the following arguments:
  * `robot_base`: (Required)

