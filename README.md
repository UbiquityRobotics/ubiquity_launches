# loki_robot

This `loki_robot` package contains various ROS launch and
parameter files.

The launch files are divided into two broad categories:

* Node launch files: A node launch file will launch a single
  ROS node.

* Confederation launch files.  A confederation launch file
  will launch a multitude of launch ROS nodes.

The overall directory architecture of this repository is:

        REPO_NAME/
            CMakeLists.txt # Standard CMakeList for catkin_make
            Package.xml    # Standard ROS package manifest
            README.md      # Documentation for everything
            includes/      # Node launch stuff goes in here
	    launches/      # Confederation launch stuff goes in here

## CMakeLists.txt

The CMakeLists.txt file is pretty minimal and looks as follows:

        cmake_minimum_required(VERSION 2.8.3)
        project(PKG_NAME)	# This is the only line that changes
        find_package(catkin REQUIRED)
        catkin_package()

The actual project name that is used by the `roslaunch` command
is specified by `project(PKG_NAME)`.  While frequently the
repository name (i.e. REPO_NAME) and the package name
(i.e. PKG_NAME) are the same, there is no requirement that
they be the same.  The user will by typing:

        roslaunch PKG_NAME LAUNCH_NAME

to fire off a confederation `.launch file` called LAUNCH_NAME.

## Node Launch Files
  
Each node launch is contained in its own sub-directory 
under the `includes` directory:
the following overall structure:

        REPO_NAME/
            includes/
                NODE_DIR/
                    ROS_NODE_NAME.launch.xml
                    # other required parameter files needed by node

where:

* REPO_NAME is the repository name (e.g. `loki_robot`,
  `magni_robot` etc.), and

* NODE_DIR is the directory that contains the `.launch.xml` file, and

* ROS_NODE_NAME is the ROS node name (e.g. `ros_arduino_bridge`,
  `joy`, etc.) that will show up in `rosnode list`.

By ending the file in .xml, the `roslaunch` command will not
be able to find the such a launch file via tab completion.
This is the desired behavior.  The reason why the launch file
ends in `.launch.xml` is explained in
[ROS Communication Issues #254](https://github.com/ros/ros_comm/issues/254).

The structure of a node launch file is:

        <launch>
          <node pkg="PKG_NAME" type="EXECUTABLE_NAME"
           name="ROS_NODE_NAME" args="..." />
	  <-- Optional ROS parameters. -->
	  <rosparm file="$(find PKG)/includes/NODE_DIR/YAML.yaml/>
        </launch>

where:

* `PKG_NAME` is the name of the package (specified in `CMakeLists.txt`.)

* `EXECUTABLE_NAME` is the name of the executable file (e.g. `cpp_node`,
  `python_node.py`, `bash_node.sh`, etc.)

* `ROS_NODE_NAME` is the ROS name that will identify the node in ROS
  (i.e. it will show up in `rosnode list`.)

* `NODE_DIR` is the name of the sub-directory that contains the
  .

* YAML is the name of the base name of `.yaml` file that contains
  some ROS parameters declarations.  Frequently, YAML matches
  ROS_NODE_NAME.

An example launch file is:

        <launch>
          <node pkg="ros_arduino_python" type="arduino_node.py"
           name="arduino">
          <rosparam command="load"
           file="$(find loki_robot)/includes/ros_arduino_bridge/rab.yaml" />
          </node>
        </launch>

## Confederation Launch Files

Each confederation launch file is kept in its own sub directory
with the following overall structure:

        REPO_NAME/
            CONFED_DIR/
                CONFED_NAME.launch
		rviz_CONFED_NAME.launch  # If appropriate
                # Other needed files.

The structure of a confederation launch file is:

        <launch>
          <!-- Launch each node: -->
	  <include file="$(find PKG_NAME)/includes/NODE1/NODE1.launch.xml" />
	  <!--... -->
	  <include file="$(find REPO_NAME)/includes/NODEn/NODEn.launch.xml" />
          &nbsp;
          <!-- Do any remaps here: -->
          <remap from="..." to="..." />
        </launch>

This package provides a bunch of launch files for using the Loki robot.

An example of confederation launch file is:

        <launch>
          <include
           file="$(find loki_robot)/includes/ros_arduino_bridge.launch.xml" />
          <include
           file="$(find loki_robot)/includes/robot_state_publisher.launch.xml" />
        </launch>

If it is appropriate for the use to run `rviz` to see the result
of running the robot, an associated rviz launch file should be present.
The format of an rviz launch file is:

        <launch>
          <node pkg="rviz" type="rviz" name="rviz"
           args="-d $(find PKG_NAME)/CONFED_NAME/CONFED_NAME.rviz"/>
        </launch>

Where the `.rviz` is an RVIZ configuration file saved from running
RVIZ.  This means the user does not have to guess how to see how
RVIZ is used to see the relevant data.

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


