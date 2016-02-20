# Ubiquity Launches

To run one of the executables below, do the following:

        rosrun ubiquity_launches PROGRAM_NAME

where, `PROGRAM_NAME` is one of the executables below.

Please note that tab completion can reduce typing:

        rosrun ub[Tab]iquity_l[Tab]aunches rasp[Tab]icam_[Tab]view

Please learn how to use tab complete, it will reduce the amount of
typing and frustration.

The following executables are available in `bin`:

* `keyboard_drive`: Start the keyboard driver for the robot.

* `keyboard_navigate`: Start the keyboard driver for the robot.

* `loki_joystick_teleop`: Run Loki with a joystick.

* `loki_local_costmap`: Cause Loki to collect local costmap.

* `loki_raspicam90`: Run Loki camera at 90 frames per second.

* `loki_rviz_local_costmap`: Show Loki local costmap in RViz.

* `loki_rviz_sonar`: Show the Loki sonars in RViz

* `platform_probe`: Return the kind of platform the current computer connected to.

* `raspicam`: Fire up a Raspberry Pi Camera on a Raspberry Pi processor.

* `raspicam_view`: Start the Raspberry Pi camera and show its output on the screen.

* `robot_base`: Run Robot Base Stack

* `roslauncher`: Helper program to launch ROS launch files.

* `yaml2launch.py`: An experimental program to convert a `.yaml` into a `.launch.xml` file.

The following launch file directories are available:

* `m_fiducial_slam`:
  Start Fiduical SLAM (Simultaneous Localication And Mapping)

* `m_joystick_teleop`:
  Start joystick remote control nodes.

* m_keyboard_drive: (No Summary Available)

* m_keyboard_navigate: (No Summary Available)

* `m_move_base`:
  

* m_move_base_view: (No Summary Available)

* `m_raspicam_raw`:
  Start nodes needed to support the Raspberry Pi camera.

* `m_raspicam_view`:
  Start all the nodes to view output of Raspberry Pi camera.

* `m_robot_base`:
  Start loki base nodes.

* `m_robot_base`:
  Start the Magni base nodes.

* m_robot_base: (No Summary Available)

* m_robot_base: (No Summary Available)

* n_amcl: (No Summary Available)

* `n_bus_server`:
  Launch a node to control the serial port.

* `n_camera`:
  Bring up a Raspberry Pi camera via GStreamer.

* `n_cmd_vel_mux`:
  

* `n_fiducial_detect`:
  Node that detects fiducial markers in images.

* `n_fiducial_slam`:
  Convert fiducial messages into a map and a localize.

* `n_image_uncompress`:
  Uncompress an image stream.

* `n_image_uncompress`:
  Uncompress an image stream.

* `n_joy`:
  Connect to a joystick node.

* n_keyboard_navigate: (No Summary Available)

* `n_map_server`:
  Start a ROS map_server node.

* `n_motor_node`:
  Start Magni motor controller software node.

* `n_move_base`:
  Run the ROS move_base node.

* `n_raspicam`:
  Start a node to read the Raspberry Pi camera.

* `n_relay`:
  Relay messages from one topic to another.

* `n_robot_state_publisher`:
  Launch the ROS `robot_state_publisher` node.

* `n_rqt_image_view`:
  Run `rqt_image_view` to view camera output on a screen.

* `n_rviz`:
  Start `rviz` with an optional `.rviz` file.

* `n_sleep_forever`:
  Start a process that sleeps for forever.

* `n_spawner`:
  Runs the ROS spawn node.

* `n_teleop_twist_joy`:
  Launch the ROS `teleop_twist_joy` node.

* n_teleop_twist_keyboard: (No Summary Available)

* `rviz_local_costmap`:
  Show local costmap in RViz.

* `rviz_sonar`:
  Show sonars in RViz.

## Executables

### `keyboard_drive` Executable:

summary here

### `keyboard_navigate` Executable:

summary here

### `loki_joystick_teleop` Executable:

This program is run on the robot and starts up a Loki platform
with a PS3/XBox game controller to control robot motion.

### `loki_local_costmap` Executable:

This program is run on the robot and starts up a Loki platform
that starts up robot that is running the both the PS3/XBox
joystick nodes and the fiducial detection and slam nodes.
The file is focused on generating a local cost map for viewing
using the `loki_rviz_local_costmap` program.

### `loki_raspicam90` Executable:

This program will start the raspicam node at 90 frames per second.

### `loki_rviz_local_costmap` Executable:

This program is run on the laptop/desktop and brings up RViz
in a mode that shows the robot, sonar sensors and local cost map.

### `loki_rviz_sonar` Executable:

This program is run on the laptop/desktop and brings up RViz
in a mode that shows the robot sonar sensors.

### `platform_probe` Executable:

The `roslauncher1` program needs to be able to query a computer
find out what kind of robot it is.  This program will return a
string of the form:

platform:=PLATFORM_NAME

where PLATFORM_NAME is one of `sim`, `loki`, `botvac`, or `magni`.

### `raspicam` Executable:

This program will start the Raspberry Pi camera attached to your
robot.  It does not bring up a viewer.  If you want the viewer as
well, please use the `raspicam_view` program instead.

### `raspicam_view` Executable:

This program will start the Raspberry Pi camera on the robot and
show the resulting images on the laptop/desktop machine.  This program
uses `rqt_image_view` view the image.  Due to race conditions, the
the `rqt_image_view` program may come up before all image topics
are present.  When this happens please click on the refresh button
(a green clockwise arrow) to update the available image topics.
Please select the topic entitled `/n_raspicam/image/camera/compressed`
to view a reasonably real-time image coming out of the Raspberry Pi
camera.

### `robot_base` Executable:


### `roslauncher` Executable:

This program is typically embedded in a short shell script that looks as follows:

\#!/bin/bash

rosrun ubiquity_launches roslauncher XXX.launch.xml

where `XXX.launch.xml` is structure launch file to be launched.

`roslauncher` does the following:

* It uses the `ROS_MASTER_URI` enviroment variable to determine
the DNS host name for the robot.

* It determins the kind of robot platform that the robot is.
For example, `loki`, `sim` (for simulator), `botvac`,
`magni`, etc.

* It sets up X11 forwarding between the current computer (which
usually has a display) and the robot (which usually does not.)
X11 forwarding always the robot to remotely open windows on
the current computer.

* It starts the top level launch file (e.g. `XXX.launch.xml`
in the example above.

### `yaml2launch.py` Executable:

This is an experimental program that does not work yet.  Please move along.

## Launch File Directories

### `m_fiducial_slam` Launch File Directory

The launch file for this directory fires off the fiducial
slam subsystem.  This causes the `move_base` node to be
loaded so that local cost maps can be generated.
This launch file requires a `robot_platform` argument to specify
which robot base is being used (e.g `loki`, `magni`, etc.)

This launch file has the following arguments:

* robot_platform (Required):
  The robot base (e.g. "magni", "loki", etc.) to use.

* mapping_mode (Optional, default: 'false'):
  Set to `true` to force mapping and `false` to disable
  mapping.

### `m_joystick_teleop` Launch File Directory

The launch file for this directory fires off the joystick
nodes to support the wireless PS2/XBox game controller for
driving the robot around.  This launch file requires a
`robot_platform` argument to specify which robot platform is being
used (e.g `loki`, `magni`, etc.)

This launch file has the following argument:

* robot_platform (Required):
  The robot platform (e.g. "magni", "loki", etc.)

### `m_keyboard_drive` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_keyboard_navigate` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_move_base` Launch File Directory



This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

* map_file (Optional, default: ' $(env TURTLEBOT_STAGE_MAP_FILE)'):

* initial_pose_x (Optional, default: '2.0'):
  Initial X position of robot in simultion.

* initial_pose_y (Optional, default: '2.0'):
  Initial Y position of robot in simultion.

* initial_pose_a (Optional, default: '0.0'):
  Initial angular position of robot in simultion (radians).

### `m_move_base_view` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_raspicam_raw` Launch File Directory

The launch file for this directory starts the
Raspberry Pi camera node and a node that uncompress the output that
comes out of the Raspberry Pi GPU (Graphical Processing Unit.)

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_raspicam_view` Launch File Directory

Start the Raspberry Pi camera and enough other nodes to
view the output of the camera.

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_robot_base` Launch File Directory

This launch file launches the core Loki stack

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_robot_base` Launch File Directory

This node fires up the various nodes to operate the
Magni robot platform.

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_robot_base` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

### `m_robot_base` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

* world_file (Optional, default: ' /opt/ros/indigo/share/turtlebot_stage/maps/stage/maze.world'):

* base (Optional, default: '$(optenv TURTLEBOT_BASE kobuki)'):

* stacks (Optional, default: '$(optenv TURTLEBOT_STACKS hexagons)'):

* 3d_sensor (Optional, default: '$(optenv TURTLEBOT_3D_SENSOR kinect)'):

### `n_amcl` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

* node_name (Optional, default: 'n_amcl'):

* use_map_topic (Optional, default: 'false'):

* scan_topic (Optional, default: 'scan'):

* initial_pose_x (Optional, default: '0.0'):

* initial_pose_y (Optional, default: '0.0'):

* initial_pose_a (Optional, default: '0.0'):

* odom_frame_id (Optional, default: 'odom'):

* base_frame_id (Optional, default: 'base_footprint'):

* global_frame_id (Optional, default: 'map'):

### `n_bus_server` Launch File Directory

The launch file for this directory starts the Ubiquity Robots
[`bus_server`](https://github.com/UbiquityRobotics/bus_server)
package that interfaces to the robot serial port.

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

### `n_camera` Launch File Directory

The launch file for this directory starts the camera node
using the ROS [`gscam`](http://wiki.ros.org/gscam) package.
This launch file is **depricated**.  You should use `n_raspicam`
launch file directory instead.

This launch file has the following arguments:

* robot_platform (Required):
  The robot base to use (e.g. "Magni" and "Loki".)

* WIDTH (Optional, default: '1280'):
  The width of the image in pixels.

* HEIGHT (Optional, default: '960'):
  The height of the image in pixels.

* FPS (Optional, default: '30/1'):
  Frame rate in frames per second. (must be an integer.)

* camera_node (Optional, default: 'camera_node'):
  The name of the camera node.

* calibration_file (Optional, default: '$(arg robot_platform).yaml'):
  The `.yaml` for camera calibration.

* respawn (Optional, default: 'false'):
  If `true` automatically respawn the node if it fails;
  otherwise just let it fail.

* DEVICE (Optional, default: '/dev/video0'):
  The device file to access for the camera.

### `n_cmd_vel_mux` Launch File Directory



This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

### `n_fiducial_detect` Launch File Directory


The launch file for this directory starts the fiducial detection
portion of the Ubiquty Robots
[fiducial](https://github.com/UbiquityRobotics/fiducials)
that is used to detect ceiling fiducials for localization.
It will in images from an image topic and genarate a topic
that contains information about each fiducial in the image.

This launch file has the following arguments:

* camera (Optional, default: '/camera'):
  The ROS topic to subscribe for for camera images.

* image (Optional, default: 'image_rect'):
  The ROS sub-topic to fetch the rectangular image from.

* fiducial_len (Optional, default: '0.2'):
  The length of one fiducial edge in meters.

* undistort_points (Optional, default: 'false'):
  If `true`, the points will be undistorted
  otherwise they will be left distorted.

### `n_fiducial_slam` Launch File Directory

This node will receive fiducial messages and create
a map of fiducial locations and fiducial orientations.  Furthermore,
node will generate a best estimate of the robot location based
on the fiducial map.

This launch file has the following arguments:

* camera_frame (Required):
  The name of the TF camera frame.

* map_frame (Required):
  The name of the TF map frame.

* odom_frame (Required):
  The name of the pose frame.

* pose_frame (Required):

* mapping_mode (Optional, default: 'false'):
  Set to `true` to enable mapping and `false` to
  disable mapping.

### `n_image_uncompress` Launch File Directory

This launch file directory will start a node that takes
a compressed image message stream and converts it to an uncompressed
image stream.

This launch file has the following argument:

* camera_name (Required):
  The base name of the camera that produces compressed
  images.  The compress input comes in on `.../image` and the output
  comes out on `.../image_raw`.

### `n_image_uncompress` Launch File Directory

This launch file directory will start a node that takes
a compressed image message stream and converts it to an uncompressed
image stream.

This launch file has the following arguments:

* camera_name (Required):
  The base name of the camera that produces compressed
  images.  The compress input comes in on `.../image` and the output
  comes out on `.../image_raw`.

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

### `n_joy` Launch File Directory

This library launch directory will launch a node that
starts the ROS [joy](http://wiki.ros.org/joy) node.  This node
interfaces to a joystick device (e.g. XBox360 controller, PS3
controller, etc.)

This launch file has no arguments.

### `n_keyboard_navigate` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

### `n_map_server` Launch File Directory

The launch file for this directory starts the ROS
[`map_server`](http://wiki.ros.org/map_server) node.

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

* map_file (Optional, default: '$(arg ul)/n_$(arg node)/params/$(arg robot_platform).yaml'):
  The `.yaml` map file to use for the map server.  This also
  assumes that there is a matching `.png` file of the same name.

* node_name (Optional, default: 'n_$(arg node)'):
  The ROS node name.

### `n_motor_node` Launch File Directory

This library launch file starts the Magni motor controller
node that controls the robot base via a serial port.

This launch file has the following argument:

* node_name (Optional, default: 'n_$(arg node)'):
  The name to assign to the node.

### `n_move_base` Launch File Directory

The launch file for this directory starts the ROS
[`move_base`](http://wiki.ros.org/move_base) node.

This launch file has no arguments.

### `n_raspicam` Launch File Directory

This launch file directory is responsible for
launching a node to read images from the Raspberry Pi camera.

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

* camera_frame_id (Optional, default: ''):

* camera_info_url (Optional, default: 'file://$(arg root)/n_$(arg node)/params/$(arg robot_platform).yaml'):

* camera_name (Optional, default: 'raspicam'):
  The name of the camera topic.

* framerate (Optional, default: '30'):

* height (Optional, default: '480'):
  The image height in pixels.

* quality (Optional, default: '20'):
  The image quality after compression wher 1 is low
  quality and 100 is high quality.

* srrc_publishing_mode (Optional, default: '0'):

* tf_prefix (Optional, default: ''):
  A prefix for the various ROS TF frame identifiers.

* width (Optional, default: '640'):
  The image width in pixels.

* output_prefix (Optional, default: '/camera'):

### `n_relay` Launch File Directory

This launch file directory will start a node that
runs the ROS [relay](http://wiki.ros.org/topic_tools/relay) node
that forwards messages from one topic to another one.

This launch file has the following arguments:

* in_topic (Required):
  The input topic name.

* out_topic (Required):
  The output topic name.

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'robot'):

* node_name (Optional, default: 'n_$(arg node)'):
  The name to assign to the node.

* unreliable (Optional, default: 'False'):
  Set to `True` to negociate an unreliable connection
  for inbound data; other set to `False` for a reliable connection.

* lazy (Optional, default: 'False'):

### `n_robot_state_publisher` Launch File Directory

The launch file for this directory starts the ROS
[`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher)
node.  This launch file selects the `.urdf` file based upon
the `robot_platform` argument.  The URDF files are stored in
`...n_robot_state_publisher/urdf/{robot_platform}.urf`.

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

* node_name (Optional, default: 'n_$(arg node)'):
  The name of the ROS node.

### `n_rqt_image_view` Launch File Directory

This runs the `rqt_image_view` node to view a camera
output stream.

This launch file has the following arguments:

* image (Required):
  The topic to view for the image stream.

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

### `n_rviz` Launch File Directory

Runs the `rviz` robot visualization tool with an an
optional argument that specifies a `.rviz` file.

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

* rviz_file (Optional, default: '$(arg ul)/n_rviz/rviz/robot_navigation.rviz'):

### `n_sleep_forever` Launch File Directory

The `robot_upstart` package will run a service that
runs a ROS `.launch` file at start up.  By running this
program, it has the side-effect of starting `roscore` and keeping
`roscore` running.


This launch file has no arguments.

### `n_spawner` Launch File Directory

This launch file directory will start a node that
runs the ROS [spawner](http://wiki.ros.org/controller_manager) node
provides a hard realtime loop to control a robot mechanism.

This launch file has the following arguments:

* arguments (Required):
  The arguments to pass into the spawner.

* node_name (Optional, default: 'n_$(arg node)'):
  The name to assign to the node.

### `n_teleop_twist_joy` Launch File Directory

The launch file for this directory starts the ROS
[`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) node.

This launch file has the following argument:

* robot_platform (Required):
  The robot platofrm (e.g. "magin", "loki", etc.)

### `n_teleop_twist_keyboard` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):

* machine_host (Required):

* machine_user (Required):

* machine_name (Optional, default: 'viewer'):

### `rviz_local_costmap` Launch File Directory

The launch file for this directory starts the ROS RViz
in a mode that shows a local costmap.

This launch file has the following argument:

* robot_platform (Required):
  The robot platform (e.g. "magni", "loki", etc.)

### `rviz_sonar` Launch File Directory

This launch file directory will launch RViz so that it
is showing the results of the sonars.

This launch file has the following argument:

* robot_platform (Required):
  The robot platform (e.g. "magni", "loki", etc.)

