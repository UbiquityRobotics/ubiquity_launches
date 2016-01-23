# `ubiquity_launches` Launch File Repository

[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_launches.svg?branch=master)](https://travis-ci.org/UbiquityRobotics/ubiquity_launches)

`ubiquity_launches` is a ROS `git` repository that contains ROS
`.launch` files and other associated ROS robot configuration files
(e.g. `.yaml`, `.urdf`, etc.)  In addition, there are shell
scripts that invoke the launch files.

A  [ROS `.launch` file](http://wiki.ros.org/roslaunch) is used
used to start and configure one or more
[ROS nodes](http://wiki.ros.org/Nodes), which when properly
configured will result in the desired robot behavior.

By imposing some structure on the organization of these
`.launch` files, we improve the ability to reproduce
robot behaviors among all robot developers who share the
use of the `ubiquity_launches` launch files.

Both the shell scripts and the ROS launch files have some
additional documentation structure.  There is a program called
[`generate_launches_md.py`](generate_launche_md.py)
that scans the `ubiquity_launches` repository and generates
a single document called [`launches.md`](launches.md).
This document provides a single location to find all of
documentation for the `ubiquity_launches` repository.
The format of the `launches.md` document is in
[markdown format](https://en.wikipedia.org/wiki/Markdown)

When `launches.md` file is served up by a git repository
management site (e.g.
[github](https://en.wikipedia.org/wiki/GitHub),
[bitbucket](https://bitbucket.org/), etc.)
it is converted into [HTML](https://en.wikipedia.org/wiki/HTML)
so that it is easier to read in a web browser.
Please view
[`launches.md`](launches.md) to see how the generated
`ubiquity_launches` documentation is rendered in a web browser.

## Download and Install `ubiquity_launches` Repository

The `ubquity_launches` repository is cloned into your catkin
workspace via the following:

        cd $ROS_CATKIN_WS/src
        git clone https://github.com/UbiquityRobotics/ubiquity_launches.git

It is strongly recommended that the `bin` directory of the
`ubiquity_launches` repository be placed in you execution path.
The preferred way to do this is by adding the following
somewhat arcane bash script code to your `~/.bashrc` file:

        # Only `source /opt/ros/indigo/setup.bash` if we have not already
        # done so.  We assume that this script does not change very often:
        if [ -d "/opt/ros/indigo/bin" ] ; then
            case ":$PATH:" in
            *:/opt/ros/indigo/bin:*) ;;
            *) source /opt/ros/indigo/setup.bash ;;
            esac
        fi

        # Put `.../ubiquity_launches/bin` in the path if it exists:
        if [ -d $ROS_CATKIN_WS/ubiquity_launches/bin ] ; then
            case ":$PATH:" in
            *:$ROS_CATKIN_WS/ubiquity_launches/bin:*) ;;
            *) export PATH=$PATH:$ROS_CATKIN_WS/ubiquity_launches/bin ;;
            esac
        fi

When you are done editing `~/.bashrc`, please run the command:

        source ~/.bashrc


## Overall Architecture

The current structure of the `ubiquity_launch` repository is
broken in a bunch of sub directories which have the following
naming conventions:

The top level directory structure is as follows:

        repository_name/
            README.md    # This document
            launches.md  # Aggregated 
            bin/         # Executable shell scripts
            n_*/         # Single ROS node launch files
            m_*/         # Multiple ROS node launch files
            rviz_*/      # RViz specific launch files

Where:

* `README.md`:  `README.md` is this documentaton file.

* `launches.md`: `launches.md` is a generated documentation file.

* `bin`: The `bin` directory contains a bunch of executables shell
  scripts that typically fire off a launch file.  It is expected
  that you will place this `bin` directory in your path.

* `n_*`: The `n_*` directories contain the launch files and
  configuration files needed to launch a single ROS Node.

* `m_*`: The `m_*` directories will launch Multiple ROS nodes.

* `rviz_*: The `rviz_*` directories are used to launch the RViz
  program configured to view a corresponding robot program.
  These launch files are typically executed on your laptop/desktop,
  since most robots do not have a display head.

Other directories and files will be added as needed.

## The `bin` Sub-Directory

The `bin` sub-directory contains a bunch of executables -- mostly
shell scripts.

The structure of an executable shell script in the `bin` sub-directory is:

        #!/usr/bin/env bash

        ##Summary: One line summary of shell script.
        ##Overview:
        ##  Line 1 of multi-line documentation.
        ##  ...
        ##  Line N of multi-line documentation.

        roslaunch ...

The first line specifies that the file is a
[bash](https://en.wikipedia.org/wiki/Bash_%28Unix_shell%29) script.

All lines that start with `##` are scanned into the documentation
in `launches.md`.

The line:

        ##Summary: One line summary of shell script.

provides a single line summary of what the shell script is supposed to do.

The lines:

        ##Overview:
        ##  Line 1 of multi-line documentation.
        ##  ...
        ##  Line N of multi-line documentation.

provide a fuller description of the what the script file does.

The remaining lines are `bash` shell script commands.
Many (but not all) script files will contain a `roslaunch ...`
command that causes ROS process a ROS `.launch` file.

## ROS Launch File Issues

There are several issues about ROS launch files that need to
be discussed:

* The `<arg>` tag is used heavily needs to be fully understood.

* There are two common launch file suffixes -- `.launch` and
  `.launch.xml`.

* Launch file parameterization allows the same launch files
  to be used for different robot platforms and configurations.

* How to add documentation to the a ROS launch file.

### The `<arg>` Tag

Some documentation for ROS launch files can be found by following
the hypertext links below:

* [ROS Launch Overview](http://wiki.ros.org/roslaunch) provides an
  overview of the ROS launch file architecture.

* [ROS launch XML file format](http://wiki.ros.org/roslaunch/XML)
  provides documentation of the XML format used for writing
  ROS launch files.

* [ROS launch `<arg>` tag](http://wiki.ros.org/roslaunch/XML/arg)
  is the documentation for the `<arg ... >` tag.

The `<arg>` tag is singled out because it is heavily used in the
launch files to pass information between launch files.
If you do not understand the `<arg>` tag, you will not understand
the `ubiquity_launches` launch files.

The `<arg>` tag has three forms:

* `<arg name="required" />`: This specifies a launch file input name.
  Think of this as a manditory and required input argument variable
  for launch file.

* `<arg name="optional" default="value" />`: This specifies a launch
  file input name with a default value that will be used if not
  is specified at "call" time.  Think of this as an optional input
  argument for a launch file.

* `<arg name="foo" value="bar" />`: This form has two different usages.
  When at the level immediately inside of a `<launch> ... </launch>`
  pair, this form defines a convenience value that can be used to
  improve overall legibility in a launch file.  Think of this as a
  kind of a macro definition.  The second form occurs immediately
  inside of a `<include> ... </include>` pair.  This form is for
  passing explicit arguments into another launch file.

Huh? What is going on here?  Let's do some examples!  The example
below us Python as a metaphor for what is going on.  If you
do not know some basic Python, well, at least we tried.

Here is a chunk of Python code that defines a routine:

        def n_fiducial_slam(robot_base, fiducial_size=".150"):
          short = "a somewhat long string"

This function is named `n_fiducial_slam` and has two arguments --
`robot_base` and `fiducial_size`.  `robot_base` is a required
argument which if not present at routine call time will cause
a run-time error.  `fiducial_size` is an optional argument that
does not need to specified in the routine call, but it can be
specified if you want.  `short` is a local variable that can
be used to reduce typing.  The corresponding launch file syntax is:

        <launch>
          <!-- Required arguments: -->
          <arg name="robot_base" />
            <!--robot_base: Robot base (e.g. "magni", "loki", etc.) to use. -->
          
          <!-- Convenience arguments: -->
          <arg name="short value="a somewhat long string" />
            <!--short_value: A short name for a longer string. -->

          <!-- Optional arguments: -->
          <arg name="fiducial_size" default=".200" />
            <!--fiducial_size: Width of fiducial measured in meters. -->
          ...
          
        </launch>

The `<include> ... </include>` tag pair is how one launch
file accesses another launch file.  It is similar to a
routine call.  In Python, the following line:

        n_fiducial_detect("loki", fiducial_size=".200")

would be written in launch file syntax as:

        ...
        
        <include file=".../n_fiducial_detect.launch">
          <arg name="robot_base" value="loki" />
          <arg name="fiducial_size" value=".200" />
        </include>
        
        ...

Now that you know how to set an argument, the only other
issue is how to access it.  That is done using substitution
arguments.  The syntax is:

        $(arg name)

where `name` is the argument name.  Using the "call" example
above, the following string:

        "base=$(arg robot_base) size=$(arg fiducial_size) short='$(arg short)'"

would expand to:

        "base=loki size=.200 short='a somewhat long string'"

The substitution syntax can only occur inside of XML attribute strings.

Finally, you pass arguments into launch files from the `roslaunch`
command via the following syntax:

        roslaunch ubiquity_launches n_fiducial_detect robot_base:="magni"

It is the `:=` is detected and conceptually converted into an
`<arg>` tag.  The line above would be represented in a launch
file as:

        <include file="$(find ubiquity_launches)/.../n_fiducial_detect.launch">
          <arg name="robot_base" value="magni" />
        </include>

Hopefully this explanation of the `<arg>` tag is a little
more informative that the official ROS documentation.

### Launch File Suffixes:

There are two ROS launch file suffixes:

* `.launch`: This launch file will be discovered by the `roslaunch`
  command via tab completion.

* `.launch.xml`: This launch file will not be discovered by
  the `roslaunch` command tab completion facility.

The [`roslaunch`](http://wiki.ros.org/roslaunch) command
has the following basic structure:

        roslaunch ROS_PACKAGE LAUNCH_NAME.launch

Basically, the `roslaunch` command searches a given ROS package
(i.e. `ROS_PACKAGE`) for a `.launch` file (i.e. `LAUNCH_NAME.launch`.)
One neat thing about `roslaunch` is that it implements
[tab completion](https://en.wikipedia.org/wiki/Command-line_completion]
whereby it will reduce overall typing by allowing you few
characters of the package name and/or launch file name followed
by a tab character to cause the `roslaunch` to fill in as
many unambiguous characters as possible.  When it comes to finding
`.launch` files, `roslaunch` recursively visits all of the
directories and sub-directories in a ROS package and identifies
every file that ends in `.launch`.  It does not matter what
package sub-directory the .launch file is in, `roslaunch` will
find it.  It is really that simple. 

A robot launch file repository will have many launch files.
Many of these file are likely to only be used via
the `<include>` tag in some other launch files.  These
launch files use the `.launch.xml` suffix, so that when
you are using tab completion for `roslaunch`, they do not
up as one of the possible completions.  That is all that
is going on here.  To let `roslaunch` show a launch file
via tab complete, use the `.launch` suffix; otherwise,
use the `.launch.xml` suffix to keep `roslaunch` from
showing the launch file via tab completion.  It is that easy.

### Launch File Parameterization

The goal of a robot launch repository is to provide
high quality launch files that work across multiple
robot platforms and configurations.  It would be
possible to build monolithic launch files that do not
use any `<include>` directives.  The reason for not
doing that is because you would have lots of replicated
text across multiple launch files.  Fixing a problem
in one launch file would have to be manually propagated
to all the other launch files.  This would be a maintenance
nightmare.

The solution is to break the launch files into a number
of smaller launch files and create the robot configuration
via composition as described in the
[roslaunch Architecture](http://wiki.ros.org/roslaunch/Architecture).

To get additional reuse, the launch files need to be
parameterized such that the same launch file can be
used for multiple robots.

For example, the most common parameter is the robot base name.
This is called the `robot_base` parameter and it expected to be
given a robot base name (e.g. `loki`, `magni`, `botvac`, etc.)
This argument is used to select between different parameter
files (e.g. `loki.yaml` vs. `magni.yaml`, or `loki.urdf` vs.
`magni.urdf`, etc.)

### Launch File Documentation Structure

The front portion of a launch file is structured as follows:

        <launch>
          <!--Summary: One line summary of launch file -->
          <!--Overview: Multi-line overview 1
              Multi-line overview 2
              ...
              Multi-line overview N -->

          <!-- Required Arguments -->
          <arg name="input_name1" />
            <!--input_name1: Documentation about `input_name1` -->
          ...
          <arg name="input_nameN" />
            <!--input_nameN: Documentation about `input_nameN` -->

          <!-- Convenience Arguments -->
          <arg name="short_name1" value="short value 1" />
          ...
          <arg name="short_nameN" value="short value N" />

          <!-- Optional Arguments -->
          <arg name="option_name1" default="default value 1" />
            <!--optional_nameN: Documentation about `optional_nameN` -->
          ...
          <arg name="option_nameN" default="default value N" />
            <!--optional_nameN: Documentation about `optional_nameN` -->

          ...

          <!-- Rest of launch file goes here: -->

        </launch>

Each comment has the form:

        <!--NAME: Documentation text -->

where `NAME` is an alpha-numeric identifier with optional underscores.
These comments are found and processed by the `generate_launches.py`
program to extract documentation.  The `NAME` field either specifies
a launch file input argument, or the values `Summary` or `Overview`.
The documentation text is written using markdown formatting rules
(e.g. *italics*, **bold**, `fixed_pitch`, etc.)

### Launch Sub-Directory Structure

Each launch sub-directory is organized as follows:

        LAUNCH_DIR_NAME/
            launch/   # Usually one `.launch` (or `.launch.xml`) file
            params/   # One or more `.yaml` (or other) parameter files
            rviz/     # One or more `.rviz` configuration files
            urdf/     # One or more `.urdf` configuration files

It is not clear why there is an additional level of sub-directory
for each different data type, but the turtlebot launch directories
have this structure, so it was decided to copy it.  (There may be
some subtle interaction with [ROS `bloom`](http://wiki.ros.org/bloom)
that is not yet fully understood.  Alternatively, it could be
monkey see, monkey do.)



### Using `git` Branches to Experiment

There is not much to say here.  If you want to tweak things
to experiment, you can use `git` to get a copy of the files,
create a branch and modify things to your hearts content.
If you can make a the case that you configuration works
better that what is currently in the `ubiquity_launches`
git repository, please submit a pull request back to the
master `ubiquity_launches` git repository.

## Some Examples

Below are three examples:

* A single node launch file (i.e. `n_*`.)

* A multi-node launch file (i.e. `m_*`.)

* An executable script file. (i.e. `bin/*`.)

### `n_*` Example

The launch file for `n_robot_state_publisher` can be found in
`.../n_robot_state_publisher/launch/n_robot_state_publisher.launch.xml`.
(There are uninteresting minor differences.)  The file is shown below:

        <launch>
          <!--Summary: Launch the ROS `robot_state_publisher` node. -->
          <!--Overview: The launch file for this directory starts the ROS
              [`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher)
              node.  This launch file selects the `.urdf` file based upon
              the `robot_base` argument.  The URDF files are stored in
              `...n_robot_state_publisher/urdf/{robot_base}.urf`. -->
        
          <!-- Required arguments: -->
          <arg name="robot_base" />
            <!--robot_base: Robot base name (e.g. "magni", "loki", etc." -->
        
          <!-- Convenience arguments: -->
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="node"  value="robot_state_publisher" />
        
          <!-- Optional arguments: -->
          <arg name="node_name" default="n_$(arg node)" />
            <!--node_name: The name of the ROS node. -->
        
          <!-- Launch the robot state publisher:  -->
          <param name="robot_description"
           textfile="$(arg root)/n_$(arg node)/urdf/$(arg robot_base).urdf" />
          <node name="$(arg node_name)"
           pkg="$(arg node)" type="state_publisher" />
        </launch>

Note that we have a pretty standard order in which stuff is done:

1. Naturally, the `<launch>` tag is first.

2. The `<!--Summary: ... -->` one line summary is next.

3. The `<!--Overview: ... -->` multi-line summary is next.

4. The `<!-- Required arguments: -->` are next.  In this case, there
   is one required argument called `robot_base`.  `robot_base` is
   one of the more common arguments.

5. The `<!-- Convenience arguments: -->` are next.  In this case,
   both `root` and `rsp` are defined.  When `root` argument is
   fully expanded, it will be something like
   `/home/name/catkin_ws/src/ubiquity_launches`.  The `root` argument
   is almost always present in an `n_*` launch file.  The `node`
   argument is short hand for the package name.  In this case,
   the package name is `robot_state_publisher`.  Using `$arg(node)`
   takes 10 characters rather than 21 characters for
   `robot_state_publisher`.

6. The `<!-- Optional arguments: -->` comes next.  In this example,
   `node_name` is defined.  This defaults to `n_robot_state_publisher`.
   Note that the `default="n_$(arg node)"` uses one of the previously
   defined arguments.  It should be mentioned that `node_name` is
   pretty much present for every `n_*` launch file.  This lets people
   create multiple nodes using the same launch file, with each
   node having a different name.

7. The `<param ...>` is next.  It specifies the parameter name and
   a fully qualified path to the `.urdf` file.  On the Ubiquity
   standard Raspberry Pi system image with `robot_base` set to `loki`,
   `$(arg root)/n_$(arg node)/urdf/$(arg robot_base).urdf` expands to
   `/home/ubuntu/catkin_ws/ubiquity_launches/n_robot_state_publisher/urdf/loki.urdf`

8. The `<node ...>` is next.  It actually launches the ROS node.
   The node name comes from `$(arg node_name)`, which will default
   to `n_robot_state_publisher`.  The package name comes from
   `$(arg node)` which expands to `robot_state_publisher`.

9. The `</launch>` comes last.

That should give you a pretty good understanding of how an `n_*`
style launch file tends to be structured.

### `m_*` Example

The launch file for `m_joystick_teleop` can be found in
`.../m_joystick_teleop/launch/n_joystick_teleop.launch`.
(There are some uninteresting minor differences.)
The file is shown below:

        <launch>
          <!--Summary: Start joystick remote control nodes. -->
          <!--Overview: The launch file for this directory fires off the
	      joystick nodes to support the wireless PS2/XBox game controller
	      for  driving the robot around.  This launch file requires a
              `robot_base` argument to specify which robot base is being
              used (e.g `loki`, `magni`, etc.) -->
        
          <!-- Required arguments: -->
          <arg name="robot_base" />
            <!--robot_base: Base being used (e.g. "magni", "loki", etc.) -->
        
          <!-- Convenience arguments: -->
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="bsm"  value="$(arg robot_base)_serial_master" />
          <arg name="rsp"  value="robot_state_publisher" />
          <arg name="joy"  value="joy" />
          <arg name="ttj"  value="teleop_twist_joy" />
        
          <!-- Fire off each node: -->
          <include
           file="$(arg root)/n_$(arg bsm)/launch/n_$(arg bsm).launch.xml" />
        
          <include
           file="$(arg root)/n_$(arg rsp)/launch/n_$(arg rsp).launch.xml">
            <arg name="robot_base" value="$(arg robot_base)" />
          </include>
        
          <include
           file="$(arg root)/n_$(arg joy)/launch/n_$(arg joy).launch.xml" />
        
          <include
           file="$(arg root)/n_$(arg ttj)/launch/n_$(arg ttj).launch.xml">
            <arg name="robot_base" value="$(arg robot_base)" />
          </include>
        </launch>

Note that we have a pretty standard order in which stuff is done:

1. The `launch`, `<!--Summary: ...>`, `<--Overview: ...>`, and
   `<!-- Required arguments -->`, `<!-- Convenience arguments -->`
   are basically the same as the `n_*` example.  There are a few
   more convenience arguments.  Lastly, there are no 
   `<!-- Optional arguments: ->`

2. Next, we start each node.  Each node that is launched has its own
   `<include ...>` tag.  In this example, 4 nodes are nodes being
   launched, so there are for `<include ... >` tags.  Two of the
   nodes require the `robot_base` argument, so they look as follows:

          <include
           file="$(arg root)/n_$(arg rsp)/launch/n_$(arg rsp).launch.xml">
            <arg name="robot_base" value="$(arg robot_base)" />
          </include>
        
   Notice how the `file` attribibute is specified.  `$(arg root)`
   expands to `$(find ubiquity_launces)` which further expands
   to `/home/ubuntu/catkin_ws/src/ubiquity_launches`.  The `$(arg rsp)`
   is a convenience argument that expands to `robot_state_publisher`.
   When fully expanded, the full path to the desired launch file
   is found.  The `<arg ...>` specifies `robot_base` value to be
   `$(arg robot_base)` which was passed into this launch file.  Thus,
   `robot_base` is being passed through.  The launch files without
   required arguments do not need any `<arg ...>` tags.

### `bin/*` Example

Here is an example:

        #!/usr/bin/env bash
        
        ##Summary: Cause Loki to collect local costmap.
        ##Overview:
        ##  This program is run on the robot and starts up a Loki platform
        ##  that starts up robot that is running the both the PS3/XBox
        ##  joystick nodes and the fiducial detection and slam nodes.
        ##  The file is focused on generating a local cost map for viewing
        ##  using the `loki_rviz_local_costmap` program.
        
        roslaunch ubiquity_launches m_fiducial_slam.launch.xml robot_base:=loki


## Some Extra Thoughts

Right now all the files in the `.../ubiquity_launches/bin` directory
have a platform base name embedded in them (e.g. `loki_raspicam`.)
This is kind of bogus.  It causes there to be a plethora of
programs in the `bin` directory. In order to solve this problem,
we need a way to identify the platform base name separate from the
program name.

For the desktop/laptop, the patform name should just be fetch
from the ROS parameters table.  Something like `/ur/base_name`
would be fine.  Thus, `loki_view_raspicam` would simplify to
`ur_view_raspicam` and would work for the `stage`, `loki`,
`magni` and `botvac` bases.  The launch file, would set the
base via appropriate 

For the robot, we should just load the `/ur/base_name` parameter
from a known file.  There a multitude of different places to store
the base name and get it set.  We can also have a `/ur/variant`
parameter for different variants of the same basic base platform
(e.g. `botvac50`, `xv11`, etc.)  After the dust settles, the user
can just run `ur_raspicam` and it will bring up the RaspiCam
on the appropriate Loki, Magni, or BotVac platform.

