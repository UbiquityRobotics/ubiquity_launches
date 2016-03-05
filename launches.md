# Ubiquity Launches

To run one of the executables below, do the following:

        rosrun ubiquity_launches PROGRAM_NAME


where, `PROGRAM_NAME` is one of the executables below.

Please note that tab completion can reduce typing:


        rosrun ub[Tab]iquity_l[Tab]aunches rasp[Tab]icam_[Tab]view

Please learn how to use tab complete, it will reduce the amount of
typing and frustration.

The following executables are available in `bin`:

* `devel_install.sh`: 

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

* `m_keyboard_drive`:
  Launch a robot in a mode that can be driven from a keyboard.

* `m_keyboard_navigate`:
  Navigate robot by keyboard with an RViz viewer.

* `m_move_base`:
  Starts the `move_base` navigation system for a robot.

* `m_move_base_view`:
  Set up a robot base for navigation with viewing via RViz.

* `m_raspicam_raw`:
  Start nodes needed to support the Raspberry Pi camera.

* `m_raspicam_view`:
  Start all the nodes to view output of Raspberry Pi camera.

* `m_robot_base`:
  Start loki base nodes.

* `m_robot_base`:
  Start the Magni base nodes.

* `m_robot_base`:
  Set up a robot base for operation.

* m_robot_base: (No Summary Available)

* `n_amcl`:
  Launch the Adaptive Monte Carlo Localization node.

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

* `n_joint_state_publisher`:
  Launches joint_state_publisher node.

* `n_joy`:
  Connect to a joystick node.

* `n_keyboard_navigate`:
  Start keyboard navigation node (currently broken.)

* `n_map_server`:
  Start a ROS map_server node.

* `n_motor_node`:
  Start Magni motor controller software node.

* `n_move_base`:
  Run the ROS move_base node.

* `n_navigation_velocity_smoother`:
  Launches navigation_velocity_smoother node.

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

* `n_stage_ros`:
  Launches stage robot simulation envirnoment for ROS.

* `n_teleop_twist_joy`:
  Launch the ROS `teleop_twist_joy` node.

* `n_teleop_twist_keyboard`:
  Launches the ROS `teleop_twist_joy/teleop_node` node.

* `rviz_local_costmap`:
  Show local costmap in RViz.

* `rviz_sonar`:
  Show sonars in RViz.

## Executables

### `devel_install.sh` Executable:


### `keyboard_drive` Executable:

summary here

* m_keyboard_drive
  * m_robot_base
    * n_joint_state_publisher
    * n_relay
    * n_stage_ros
    * n_cmd_vel_mux
    * n_robot_state_publisher
  * n_teleop_twist_keyboard

### `keyboard_navigate` Executable:

summary here

* m_keyboard_navigate
  * m_move_base_view
    * m_move_base
      * m_robot_base
        * n_joint_state_publisher
        * n_relay
        * n_stage_ros
        * n_cmd_vel_mux
        * n_robot_state_publisher
      * n_navigation_velocity_smoother
      * n_move_base
      * n_map_server
      * n_amcl
    * n_rviz
  * n_keyboard_navigate

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

* m_raspicam_raw
  * n_raspicam
  * n_image_uncompress

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

* m_raspicam_view
  * m_raspicam_raw
    * n_raspicam
    * n_image_uncompress
  * n_rqt_image_view

### `robot_base` Executable:


* m_robot_base
  * n_joint_state_publisher
  * n_relay
  * n_stage_ros
  * n_cmd_vel_mux
  * n_robot_state_publisher

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

Set up a robot so that it can be driven from the keyboard
on the development machine.  The ROS
[teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)
node documentation explains the keyboard controls.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* robot_host (Required):
  The DNS address for the robot.

* robot_user (Required):
  The user account on the robot to use.

* viewer_host (Optional, default: 'localhost'):
  The DNS address for the viewer machine with a display.

* viewer_user (Optional, default: ''):
  The user account on the display computer to use.

### `m_keyboard_navigate` Launch File Directory

Configure the robot for navigation with a keyboard
and start of RViz to show progress.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* robot_host (Required):
  The DNS address for the robot.

* robot_user (Required):
  The user account on the robot to use.

* viewer_host (Optional, default: 'localhost'):
  The DNS address for the viewer machine with a display.

* viewer_user (Optional, default: ''):
  The user account on the display computer to use.

### `m_move_base` Launch File Directory

Start the `move_base` navigation system for the currently
selected robot.

This launch file has the following arguments:

* robot_platform (Required):

* robot_host (Required):

* robot_user (Required):

* viewer_host (Optional, default: 'localhost'):

* viewer_user (Optional, default: ''):

* map_file (Optional, default: '$(arg ul)/m_robot_base/maps/stage/maze.world'):
  The stage `.world` file to use.

* initial_pose_x (Optional, default: '2.0'):
  Initial X position of robot in simultion.

* initial_pose_y (Optional, default: '2.0'):
  Initial Y position of robot in simultion.

* initial_pose_a (Optional, default: '0.0'):
  Initial angular position of robot in simultion (radians).

### `m_move_base_view` Launch File Directory

Run the nodes to required to navigate a robot platform along
with an RViz node to view what is going on.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "loki", "sim", "magni") to use.

* robot_host (Required):
  The DNS address for the robot.

* robot_user (Required):
  The user account on the robot to use.

* viewer_host (Optional, default: 'localhost'):
  The DNS address for the viewer machine with a display.

* viewer_user (Optional, default: ''):
  The user account on the display computer to use.

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

Launches all the nodes needed to bring up basic driving
of a robot platform.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* robot_host (Required):
  The DNS address for the robot.

* robot_user (Required):
  The user account on the robot to use.

* viewer_host (Optional, default: 'localhost'):
  The DNS address for the viewer machine with a display.

* viewer_user (Optional, default: ''):
  The user account on the display computer to use.

### `m_robot_base` Launch File Directory

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "loki", "sim", "magni") to use.

* robot_host (Required):
  The DNS address for the robot.

* robot_user (Required):
  The user account on the robot to use.

* viewer_host (Optional, default: 'localhost'):
  The DNS address for the viewer machine with a display.

* viewer_user (Optional, default: ''):
  The user account on the display computer to use.

* base (Optional, default: 'kobuki'):
  The kind of robot base (e.g. "create", "kobuki", "Rhoomba", ...).

* stacks (Optional, default: 'hexagons'):
  The kind of stack on the robot (e.g. "circles", "hexagons").

* 3d_sensor (Optional, default: 'asus_xtion_pro'):
  The kind of 3D sensor (e.g. "kinect, "asus_xtion_pro").

* world_file (Optional, default: '$(arg ul)/m_robot_base/maps/stage/maze.world'):
  The `.world` file to construct the robot simulation environment.
  Note that the currently the world file must be in a in a directory called
  `.../maps/stage/` which need a bunch of .png files, .yaml files, etc.
  Good luck finding any useful documentation.

### `n_amcl` Launch File Directory

This node will launch the
[Adaptive Monte Carlo Localization](http://wiki.ros.org/amcl) node.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* node_name (Optional, default: 'n_amcl'):
  The name to assign to this ROS node.

* use_map_topic (Optional, default: 'false'):
  `true` enable the use of the map topic.

* scan_topic (Optional, default: 'scan'):
  The name of the LIDAR scan topic to subscribe to.

* initial_pose_x (Optional, default: '0.0'):
  The initial X location of the robot.

* initial_pose_y (Optional, default: '0.0'):
  The initial Y location of the robot.

* initial_pose_a (Optional, default: '0.0'):
  The initial angle of the robot.

* odom_frame_id (Optional, default: 'odom'):
  The Odometry TF frame id.

* base_frame_id (Optional, default: 'base_footprint'):
  The robot base TF frame id.

* global_frame_id (Optional, default: 'map'):
  The global TF frame id.

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
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

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

This launch file has the following arguments:

* camera_name (Required):
  The base name of the camera that produces compressed
  images.  The compress input comes in on `.../image` and the output
  comes out on `.../image_raw`.

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

### `n_joint_state_publisher` Launch File Directory

This will launch a joint_state_publisher node.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* joint_states_topic (Optional, default: 'joint_states'):
  The topic to publish joint states on.

* node_name (Optional, default: 'n_$(arg jsp)'):
  The name to assign to this node.

* rate (Optional, default: '10'):
  The rate at which joint states are published.

* use_gui (Optional, default: 'False'):
  If "True", pops up a GUI window that allows the joints to be changed.

### `n_joy` Launch File Directory

This library launch directory will launch a node that
starts the ROS [joy](http://wiki.ros.org/joy) node.  This node
interfaces to a joystick device (e.g. XBox360 controller, PS3
controller, etc.)

This launch file has no arguments.

### `n_keyboard_navigate` Launch File Directory

The launch file for this directory starts a
keyboard interface to the ROS
[`move_base`](http://wiki.ros.org/map_server) node.  This
code is still under development.  (Code is currently broken.)

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

### `n_map_server` Launch File Directory

The launch file for this directory starts the ROS
[`map_server`](http://wiki.ros.org/map_server) node.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

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
[`move_base`](http://wiki.ros.org/move_base) node for manageing
robot navigation.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* odom_frame_id (Optional, default: 'odom'):
  The odomentry TF id.

* base_frame_id (Optional, default: 'base_footprint'):
  The base frame TF id.

* global_frame_id (Optional, default: 'map'):
  The global frame TF id.

* odom_topic (Optional, default: 'odom'):
  The odemetry topic to subscribe to.

* laser_topic (Optional, default: 'scan'):
  The laser topic to subscribe to.

* custom_param_file (Optional, default: '$(find turtlebot_navigation)/param/dummy.yaml'):
  Not a clue.

### `n_navigation_velocity_smoother` Launch File Directory

This will launch a the
[yocs_velocity_smoother](http://wiki.ros.org/yocs_velocity_smoother) node.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* yaml_file (Optional, default: '$(arg ul)/n_$(arg nvs)/yaml/empty.yaml'):
  A `.yaml` file that can specifiy the various parameters for this node
  as an alternative to specifying them indificually.  Individual arguments
  should override the `.yaml` file.

* node_name (Optional, default: 'n_$(arg nvs)'):
  The name to use for the node:

* raw_cmd_vel_stopic (Optional, default: '$(arg node_name)/raw_cmd_vel'):
  The topic to subscribe to get the input velocity commands.

* odometry_stopic (Optional, default: '$(arg node_name)/odometry'):
  The topic to subscribe to look at to for the robot odometry.

* robot_cmd_vel_stopic (Optional, default: '$(arg node_name)/robot_cmd_vel'):

* smooth_cmd_vel_ptopic (Optional, default: '$(arg node_name)/smooth_cmd_vel'):
  The topic that is published which has the
  smoothed velocity commands.

* speed_lim_v (Optional, default: '0.8'):
  Linear velocity limit.

* speed_lim_w (Optional, default: '5.4'):
  Angular velocity limit.

* decel_factor (Optional, default: '1.5'):
  Deceleration/acceleration ratio. Useful to make deceleration
  more aggressive, for example to safely brake on robots with high inertia.

* frequency (Optional, default: '2.0'):

* robot_feedback (Optional, default: '2'):
  Specifies which topic to use as robot velocity feedback
  (0 - none, 1 - odometry, 2 - end robot commands). See hints below for more details.

### `n_raspicam` Launch File Directory

This launch file directory is responsible for
launching a node to read images from the Raspberry Pi camera.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* camera_frame_id (Optional, default: ''):
  The name of the ROS TF camera frame.

* camera_info_url (Optional, default: 'file://$(arg root)/n_$(arg node)/params/$(arg robot_platform).yaml'):
  The URL for the camera `.yaml` file.

* camera_name (Optional, default: 'raspicam'):
  The name of the camera topic.

* framerate (Optional, default: '30'):
  The frame rate between 15 and 90 as an integer.

* height (Optional, default: '480'):
  The image height in pixels.

* quality (Optional, default: '20'):
  The image quality after compression wher 1 is low
  quality and 100 is high quality.

* srrc_publishing_mode (Optional, default: '0'):
  (Not a clue what this argument does!)

* tf_prefix (Optional, default: ''):
  A prefix for the various ROS TF frame identifiers.

* width (Optional, default: '640'):
  The image width in pixels.

* output_prefix (Optional, default: '/camera'):
  The prefix for the output topics of this node.

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
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* node_name (Optional, default: 'n_$(arg node)'):
  The name to assign to the node.

* unreliable (Optional, default: 'False'):
  Set to `True` to negociate an unreliable connection
  for inbound data; other set to `False` for a reliable connection.

* lazy (Optional, default: 'False'):
  Set to `True` defer subscribing to the input topic until
  after there is at least one output topic; otherwise set to `False`
  to always subscribe to both topics.

### `n_robot_state_publisher` Launch File Directory

The launch file for this directory starts the ROS
[`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher)
node.  This launch file selects the `.urdf` file based upon
the `robot_platform` argument.  The URDF files are stored in
`...n_robot_state_publisher/urdf/{robot_platform}.urf`.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

* joint_states_topic (Optional, default: 'joint_states'):
  Topic on which joint states are published.

* node_name (Optional, default: 'n_$(arg rsp)'):
  The name of the ROS node.

* tf_prefix (Optional, default: ''):
  The text to prepend to each TF name.

* publish_frequency (Optional, default: '50.0'):
  The frequency at which to publish

* use_tf_static (Optional, default: 'true'):
  Set to true to use /tf_static latched static broadcaster.

* robot_description_file (Optional, default: '$(arg ul)/n_$(arg rsp)/urdf/$(arg robot_platform).urdf'):
  The name of a file that contains the robot
  structure in URDF format.

### `n_rqt_image_view` Launch File Directory

This runs the `rqt_image_view` node to view a camera
output stream.

This launch file has the following arguments:

* image (Required):
  The topic to view for the image stream.

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

### `n_rviz` Launch File Directory

Runs the `rviz` robot visualization tool with an an
optional argument that specifies a `.rviz` file.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

* rviz_file (Optional, default: '$(arg ul)/n_rviz/rviz/robot_navigation.rviz'):
  A file that preconfigures rviz to show navigation
  information.

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

### `n_stage_ros` Launch File Directory

Stage is an environment for simulating robot operating
in a simulated 2D environment.  This node starts up the
stage_ros/stageros program to perform this simulation.  The
[Stage Manual](http://playerstage.sourceforge.net/doc/Stage-3.2.1/modules.html)
is available elsewhere on the net.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* world_file (Required):
  The `.world` file to construct the robot simulation environment.
  Note that the currently the world file must be in a in a directory called
  `.../maps/stage/` which need a bunch of .png files, .yaml files, etc.
  Good luck finding any useful documentation.

* machine_name (Optional, default: 'robot'):
  The machine name (i.e. "robot" or "viewer")

* joint_states_topic (Optional, default: 'joint_states'):
  The topic to publish joint states on.

* node_name (Optional, default: 'n_$(arg sr)'):
  The name to assign to this node.

* rate (Optional, default: '10'):
  The rate at which joint states are published.

* use_gui (Optional, default: 'False'):
  If "True", pops up a GUI window that allows the joints to be changed.

### `n_teleop_twist_joy` Launch File Directory

The launch file for this directory starts the ROS
[`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) node.

This launch file has the following argument:

* robot_platform (Required):
  The robot platofrm (e.g. "magin", "loki", etc.)

### `n_teleop_twist_keyboard` Launch File Directory

This launch file directory will fire up a ROS node that
will talk to either a PS3 or XBox game controller module.

This launch file has the following arguments:

* robot_platform (Required):
  The robot platform (e.g. "magin", "loki", etc.)

* machine_host (Required):
  The DNS machine name (e.g. "ubuntu.local")

* machine_user (Required):
  The user account on the machine.

* machine_name (Optional, default: 'viewer'):
  The machine name (i.e. "robot" or "viewer")

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

