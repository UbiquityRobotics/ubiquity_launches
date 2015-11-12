# Ubiquity Launches

The following launch file directories are available:

* m_fiducial_slam:
  Start Fiduical SLAM (Simultaneous Localication And Mapping)

* m_joystick_teleop:
  Start joystick remote control nodes.

* m_raspicam_raw:
  Start nodes needed to support the Raspberry Pi camera.

* n_bus_server:
  Launch a node to control the serial port.

* n_camera:
  Bring up a Raspberry Pi camera via GStreamer.

* n_fiducial_detect:
  Node that detects fiducial markers in images.

* n_fiducial_slam:
  Convert fiducial messages into a map and a localize.

* n_image_uncompress:
  Uncompress an image stream.

* n_joy:
  Connect to a joystick node.

* n_loki_serial_master:
  A wrapper for launching the `n_bus_server` launch file.

* n_map_server:
  Start a ROS map_server node.

* n_move_base:
  Run the ROS move_base node.

* n_raspicam:
  Start a node to read the Raspberry Pi camera.

* n_robot_state_publisher:
  Launch the ROS `robot_state_publisher` node.

* n_teleop_twist_joy:
  Launch the ROS `teleop_twist_joy` node.

* rviz_local_costmap:
  Show local costmap in RViz.

* rviz_sonar:
  Show sonars in RViz.

## m_fiducial_slam Launch File Directory

The launch file for this directory fires off the fiducial
slam subsystem.  This causes the `move_base` node to be
loaded so that local cost maps can be generated.
This launch file requires a `robot_base` argument to specify
which robot base is being used (e.g `loki`, `magni`, etc.)

This launch file has the following arguments:

* robot_base (Required):
  The robot base (e.g. "magni", "loki", etc.) to use.

* mapping_mode (Optional, default: 'false'):
  Set to `true` to force mapping and `false` to disable
  mapping.

## m_joystick_teleop Launch File Directory

The launch file for this directory fires off the joystick
nodes to support the wireless PS2/XBox game controller for
driving the robot around.  This launch file requires a
`robot_base` argument to specify which robot base is being
used (e.g `loki`, `magni`, etc.)

This launch file has the following argument:

* robot_base (Required):
  The robot base being used (e.g. "magni", "loki", etc.)

## m_raspicam_raw Launch File Directory

The launch file for this directory starts the
Raspberry Pi camera node and a node that uncompress the output that
comes out of the Raspberry Pi GPU (Graphical Processing Unit.)

This launch file has the following argument:

* robot_base (Required):
  The robot base to use (e.g. "magni", "loki", etc.)

## n_bus_server Launch File Directory

The launch file for this directory starts the Ubiquity Robots
[`bus_server`](https://github.com/UbiquityRobotics/bus_server)
package that interfaces to the robot serial port.

This launch file has the following argument:

* robot_base (Required):
  The robot base name (e.g. "magni", "loki", etc.)

## n_camera Launch File Directory

The launch file for this directory starts the camera node
using the ROS [`gscam`](http://wiki.ros.org/gscam) package.
This launch file is **depricated**.  You should use `n_raspicam`
launch file directory instead.

This launch file has the following arguments:

* robot_base (Required):
  The robot base to use (e.g. "Magni" and "Loki".)

* WIDTH (Optional, default: '1280'):
  The width of the image in pixels.

* HEIGHT (Optional, default: '960'):
  The height of the image in pixels.

* FPS (Optional, default: '30/1'):
  Frame rate in frames per second. (must be an integer.)

* camera_node (Optional, default: 'camera_node'):
  The name of the camera node.

* calibration_file (Optional, default: '$(arg robot_base).yaml'):
  The `.yaml` for camera calibration.

* respawn (Optional, default: 'false'):
  If `true` automatically respawn the node if it fails;
  otherwise just let it fail.

* DEVICE (Optional, default: '/dev/video0'):
  The device file to access for the camera.

## n_fiducial_detect Launch File Directory


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

* fiducial_len (Optional, default: '0.146'):
  The length of one fiducial edge in meters.

* undistort_points (Optional, default: 'false'):
  If `true`, the points will be undistorted
  otherwise they will be left distorted.

## n_fiducial_slam Launch File Directory

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

## n_image_uncompress Launch File Directory

This launch file directory will start a node that takes
a compressed image message stream and converts it to an uncompressed
image stream.

This launch file has the following argument:

* camera_name (Required):
  The base name of the camera that produces compressed
  images.  The compress input comes in on `.../image` and the output
  comes out on `.../image_raw`.

## n_joy Launch File Directory

This library launch directory will launch a node that
starts the ROS [joy](http://wiki.ros.org/joy) node.  This node
interfaces to a joystick device (e.g. XBox360 controller, PS3
controller, etc.)

This launch file has no arguments.

## n_loki_serial_master Launch File Directory

The launch file for this directory starts the `n_bus_server`
node configured for the Loki platform.

This launch file has no arguments.

## n_map_server Launch File Directory

The launch file for this directory starts the ROS
[`map_server`](http://wiki.ros.org/map_server) node.

This launch file has no arguments.

## n_move_base Launch File Directory

The launch file for this directory starts the ROS
[`move_base`](http://wiki.ros.org/move_base) node.

This launch file has no arguments.

## n_raspicam Launch File Directory

This launch file directory is responsible for
launching a node to read images from the Raspberry Pi camera.

This launch file has the following arguments:

* robot_base (Required):
  The robot base being used (e.g. "magni", "loki", etc. )

* camera_frame_id (Optional, default: ''):

* camera_info_url (Optional, default: 'file://$(arg root)/n_$(arg node)/params/$(arg robot_base).yaml'):

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

## n_robot_state_publisher Launch File Directory

The launch file for this directory starts the ROS
[`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher)
node.

This launch file has the following argument:

* robot_base (Required):
  The name of the robot base (e.g. "magni", "loki", etc."

## n_teleop_twist_joy Launch File Directory

The launch file for this directory starts the ROS
[`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) node.

This launch file has the following argument:

* robot_base (Required):
  The name of the robot base (e.g. "magin", "loki", etc.

## rviz_local_costmap Launch File Directory

The launch file for this directory starts the ROS RViz
in a mode that shows a local costmap.

This launch file has the following argument:

* robot_base (Required):
  The name of the robot base (e.g. "magni", "loki", etc.)

## rviz_sonar Launch File Directory

This launch file directory will launch RViz so that it
is showing the results of the sonars.

This launch file has the following argument:

* robot_base (Required):
  The name of the robot base (e.g. "magni", "loki", etc.)

