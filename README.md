# `ubiquity_launches` Launch File Repository

`ubiquity_launches` is a ROS `git` repository that contains
ROS `.launch` files and associated robot configuration files
(e.g. `.yaml`, `.urdf`, etc.)  In addition, there are shell
scripts that invoke the launch files.

A  [ROS `.launch` file](http://wiki.ros.org/roslaunch) is used
used to start and configure one or more
[ROS nodes](http://wiki.ros.org/Nodes), which when properly
configured will result in the desired robot behavior.

By imposing some structure on the organization of these
launch repositories, we improve the ability to reproduce
robot behaviors among all robot developers who use the
`ubiquity_launches` launch files.

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

## Introduction to `ubiquity_launches`

The current structure of the `ubiquity_launch` repository is
broken in a bunch of sub directories which have the following
naming conventions:

* `bin`: The `bin` directory contains a bunch of executables shell
  scripts that typically fire off a launch file.

* `n_*`: The `n_*` directories contain the launch files and
  configuration files needed to launch a single ROS Node.

* `m_*`: The `m_*` directories will launch Multiple ROS nodes.

* `rviz_*: The `rviz_*` directories are used to launch the RViz
  program configured to view a corresponding robot program.

* others: Other directories will be added as needed.

## `bin` Sub-Directory

The format of a executable shell script in the `bin` sub-directory is:

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
command that causes ROS to launch a bunch of ROS nodes.

It is strongly recommended that the `bin` directory of
`ubiquity_launches` be placed in you execution path.
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

## Overall Architecture

Ultimately, there are a bunch of shell scripts and launch files. 
The shell scripts will typically call a launch file with one
or more launch file arguments set.

The top level directory structure is as follows:

        repository_name/
            README.md    # This document
            launches.md  # Aggregated 
            bin/         # Executable shell scripts
            n_*/         # Single ROS node launch files
            m_*/         # Multiple ROS node launch files
            rviz_*/      # RViz specific launch files

We use some simple directory naming conventions to clue
people into what kind of launch file is in a directory:

* `bin`: The `bin` directory contains a bunch of executable
  shell scripts that fire off a launch file.  It is expected
  that you will place this `bin` directory in your path.

* `n_*`: The n_* directories contain a ROS launch files and
  associated configuration files needed to launch a single
  ROS Node.

* `m_*`: The m_* directories contain a ROS launch that
  will start Multiple ROS nodes.

* `rviz_*`: The `rviz_*` directories are used to launch the
  RViz program configured to view a corresponding robot program.
  These launch files are typically executed on your laptop/desktop,
  since most robots do not have a display head.

* `view_*`: The `view_*` directories are used to launch the
  `rqt_image_view` program configured to view a specific image
  topic.  These launch files are typically executed on your
  laptop/desktop, since most robots do not have a display head.

* others: Other directories contain miscellaneous launch files.

### Top Level Directory Structure

The top level directory structure is as follows:

        repository_name/
            README.md    # Documentation for **ALL** launch files
            bin/         # Executable shell scripts
            n_*/         # Single ROS node launch files
            m_*/         # Multiple ROS node launch files
            rviz_*/      # RViz specific launch files

We use some simple directory naming conventions to clue
people into what kind of launch file is in a directory.

* `bin`: This directory contains a bunch of short shell
  scripts.  It is expected that you will choose to place
  this `bin` directory into your `$PATH`.

* `n_`: If a launch directory starts with `n_`, the associated
  launch file starts a single ROS node.  The `n` in `n_` is
  short for Node.

* `m_`: If a launch directory starts with `m_`, the associated
  launch file fires up multiple ROS nodes.  The `m` in `m_` is
  short for Multiple.

* `rviz_*`: If a launch file starts with `rviz_`, the associated
  launch file is used to launch the
  [RViz](http://wiki.ros.org/rviz) visualization tool on a
  laptop/desktop.  The launch file properly configures `rviz`.

While this structure will evolve over time, currently it is
not very complicated.




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

The `<arg>` tag is singled out because it is used in the
launch files to pass around the robot platform information.
If you do not understand the `<arg>`, you will not understand
the `ubiquity_launches` launch files.

The `<arg>` tag has three forms:

* `<arg name="required" />`: This specifies a launch file input name.
  Think of this as an argument variable for routine.

* `<arg name="optional" default="value" />`: This specifies a launch
  file input name with a default value that will be used if not
  is specified at "call" time.

* `<arg name="foo" value="bar" />`: This form has two different usages.
  When at the level immediately inside of a `<launch> ... </launch>`
  pair, this form defines a convenience value that can be used to
  improve overall legibility.  Think of this as a kind of a macro
  definition.  The second form occurs immediately inside of a
  `<include> ... </include>` pair.  This form is like passing arguments
  into a routine call.

Huh? What is going on here?  Let's do some examples!  Here is
a chunk of Python code that defines a routine:

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
          
          <!-- Convenience arguments: -->
          <arg name="short value="a somewhat long string" />
          
          <!-- Optional arguments: -->
          <arg name="fiducial_size" default=".200" />
          
          ...
          
        </launch>

The `<include> ... </include>` tag pair is how one launch
file accesses another launch file.  It is similar to a
routine call.  In python, the following line:

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

The substitution syntax can only occur inside of XML attribute
strings.

Finally, you pass arguments into launch files from the `roslaunch`
command via the following syntax:

        roslaunch ubiquity_launches n_fiducial_detect robot_base:="magni"

It is the `:=` is detected and conceptually converted into an
`<arg>` tag.  The line above would be represented in a launch
file as:

        <include file="$(find ubiquity_launches)/n_fiducial_detect.launch">
          <arg name="robot_base" value="magni" />
        </include>

Hopefully this explanation of the `<arg>` tag is a little
more informative that the official ROS documentation.

### Launch File Suffixes:

There are two ROS launch file suffixes:

* `.launch`: This launch file will be discovered by the `roslaunch`
  command.

* `.launch.xml`: This launch file will not be discovered by
  the `roslaunch` command.

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
Many of these file are not likely to only be used via
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

### Basic launch file structure:

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

Each comment of the form:

        <!--NAME: Documentation text -->

where `NAME` is an alpha-numeric identifier with optional underscores,
is processed by the `generate_launches.py` program to extract documentation
The `NAME` field either specifies a launch file input argument,
or the values `Summary` or `Overview`.   The documentation text
is written using markdown formatting rules (e.g. *italics*, **bold**,
`fixed_pitch`, etc.)

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

### Basic File Structure:

There are two primary files in this repository -- short
shell scripts and launch files.

The shell scripts are usually only a few lines and look
as follows:

        #!/usr/bin/env bash
        roslaunch ubiquity_launches LAUNCH_NAME.launch robot_base:=BASE

where:

* LAUNCH_NAME is the name of a `.launch` (or `.launch.xml`) fiel, and

* BASE is a robot base name (e.g. `loki`, `magni`, etc.)

The structure of a `.launch` (or `.launch.xml`) file is as follows:

        <!-- Comment explaining what the launch file does. -->
        <launch>
          <!-- Required Arguments -->
          <arg name="required_argument_name1" />
          <arg name="required_argument_name2" />
          ...
          
          <!-- Convenience Arguments -->
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="convenience_argument_name1" value="value1" />
          <arg name="convenience_argument_name2" value="value2" />
          ...
          
          <!-- Includes -->
          <include file="...">
            <arg name="arg1" value="value1" />
            <arg name="arg2" value="value2" />
            ...
          </include>
          ...
          
          <!-- Nodes -->
          <node pkg="package_name" type="executable_name"
           name="ros_node_name" ... >
            <!-- Optional env, remap, rosparam, and param tags -->
          </node>
          
        </launch>

The first convenience argument is almost always:

          <arg name="root" value="$(find ubiquity_launches)" />

The `root` argument provides the root path to the
`ubiquity_launches` directory.  From there the include file
can find parameter files.

### Using `git` Branches to Experiment

There is not much to say here.  If you want to tweak things
to experiment, you can use `git` to get a copy of the files,
create a branch and modify things to your hearts content.
If you can make a the case that you configuration works
better that what is currently in the `ubiqutiy_launches`
git repository, please submit a pull request back to the
master `ubiquity_launches` git repository.

## Future Possibilities

There are a number of future possibilities:

* ROS 2.0 is going to have more introspection.  At some point in
  time it is likely that there will be graphical tools to help
  assemble the nodes that make up a robot behavior.  This will
  almost certainly impact this launch repository architecture.
