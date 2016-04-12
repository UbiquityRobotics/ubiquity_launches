# `ubiquity_launches` Launch File Repository

[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_launches.svg?branch=master)](https://travis-ci.org/UbiquityRobotics/ubiquity_launches)

The `ubiquity_launches` repository provides a bunch of
utilities for developing ROS code.  In particular, it
provides a common list of programs to run robot softare
on a robot simulator or one or more actual robots.

The file [`launches.md`](launches.md) summarizes what is
available.  If you need a higher level introduction to
what is going, please continue reading.

## Development Overview

All of the Ubiquity robots use
[ROS](http://www.ros.org/) (Robot Operating System).
Under ROS, a robot application comprises a number of ROS
nodes (i.e. processes) running on one or more processors
that communicate with one another via internet protocols.

The usual method for teaching people to develop ROS robot
applications is to point them at some ROS tutorials and say
"go at it".  Unfortunately, this is a little bit dumping
buckets of blood into shark infested waters and going for
a swim.  At Ubiquity Robotics, we want robot application
developers to have a much less traumatic experience.  For
this reason, we have invested a great deal of effort to
make ROS robot application development as easy as possible.

The reality is that setting up a workable software
development environment for developing ROS applications
is actually pretty involved.  One major complication is
that the robot typically does not have a display, keyboard,
and mouse.  Even if it did, it not particularly easy to
walk around with the robot as it moves.  The more workable
solution is to develop software on a stationary platform
like a laptop or a desktop, and communicate with the robot
via a wireless internet connection.  A further complication
is that Ubiquity Robotics is using a Raspberry Pi 2 processor
with the ARM7 instruction set, whereas most laptops and
desktops use the x86 instruction set.  We have to make sure
that each machine gets the right instruction set.

The Ubiquity Robotics application development environment
assumes that there are two processors.  The robot processor
is attached to the robot and the development processor is
associated with either laptop or desktop computer. ROS must
run on the development processor as well as on the robot.

Ubiquity Robotics currently uses a
[Raspberry Pi Foundation](https://www.raspberrypi.org/)
[Raspberry Pi 2 Model B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/)
for the robot processor.  Since "Raspberry Pi 2 Model B" is a
mouthful of words, we shorten it down to a more manageable "RasPi2".
The RasPi2 is a quad-core ARM Cortex-A7 CPU with 1GB of RAM,
with 4 USB Ports, an Ethernet port, a camera interface, and
a micro-SD card.  We plug a dual-band USB WiFi Dongle into
one of the USB port slots to provide wireless connectivity
with the development processor through a commercial
off-the-shelf WiFi router.

ROS currently only runs under Ubuntu and its variants (e.g.
Kubuntu, Xubuntu, Lubuntu, etc.)  No other operating system
options are currently supported by the ROS community. Thus,
the development processor must run one of these.

The development processor must be a 64-bit x86 hardware
architecture processor.  You can either run some flavor
of Ubuntu natively on the processor, or you can run a
different operating system (e.g. Windows, MacOS, Solaris,
etc.) and run [VirtualBox](https://www.virtualbox.org/)
to run the Ubuntu operating system.  Neither 32-bit processors
nor other virtual machines (e.g. VMWare,  Parallels, etc.)
are supported.

To be consistent, the VirtualBox image that we recommend 
has [Lubuntu](/http://lubuntu.net/) installed on it.
We also install Lubuntu on the robot processor, just in case
you choose to plug a monitor/keyboard/mouse into the RasPi2.
Lubuntu is a very light weight window system that does not
over tax the RasPi2 robot processor.

We really encourage people not to cut corners when it comes
to setting up your WiFi network.  If you have heard the saying
that "a chain is only as strong as its weakest link", our
experience is that WiFi is the weak link in robotics.  We
strongly recommend that people use dual band
[802.11ac](https://en.wikipedia.org/wiki/IEEE_802.11ac)
USB WiFi dongles to provide wireless technology.  Similarly,
we recommend that you invest in a superior dual-band WiFi
router.  You can spend an enormous amount of time tracking
down and eradicating flaky WiFi issues and it is better to
simply avoid the issues by using better hardware and better
antennas.

We use a technology called
[zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking)
to provide human readable names for the various processors on
the computer network.  Each robot processor and development processor
must have a unique host name and **MUST** run zeroconf.  The
[DNS](https://en.wikipedia.org/wiki/Domain_Name_System) will
present the DNS names as `hostname.local` where, `hostname` is
the host name of the robot or development processor.

The ROS community extensively uses
[Secure Shell](https://en.wikipedia.org/wiki/Secure_Shell)
to communicate between computers.  The secure shell protocols
provide a secure and encrypted link between computers.
We require that password keys be properly set up between
the development processor and the robot processor such that
a secure link can be established without prompting the user for
a password.

Once we have secure shell properly installed, we can do three
things with it:

* The `roslaunch` program can launch ROS nodes (i.e. processes)
  via secure shell.

* We use the secure shell to do X11 protocol forwarding.
  X11 a fairly old (but reliable) technology for doing remote
  window access.  Using X11 protocol forwarding, we can debug
  a program running on the robot processor with windows that
  show up on the development processor.

* We can use secure shell to remotely mount a chunk of the
  file system from the robot processor onto the development
  processor.  This allows you to use editors and tools on
  development system to develop robot code.  There are requirements
  on how all the files are organized on both machines, but
  when all the rules are followed, you can easily build
  the same ROS packages on both your development processor and
  robot processor without any major issues.

The secure shell technology is very useful, but requires some
effort to set up properly.

Lastly, we have an entire architecture for structuring ROS launch
files.  A ROS Launch file is a sequence of one or more files in
XML format, that when fed into the `roslaunch` program cause all
of the ROS nodes needed to run a robot to be started.  The goal
of the architecture is:

* Sharable: The launch files are shared among one another.

* Platform Neutral: The launch files can be used for more than one
  robot platform.

* Complete: Only one top level robot launch file is specified to
  start a robot and any ancillary visualization tools needed for
  a robot application.

The launch files are structured to make it relatively easy to
add a new robot platform without having to rewrite dozens of
launch file.  We call this launch file architecture the *Unified
Launch File Architecture*.

The Unified Launch file architecture is based out of a ROS package
called `ubiquity_launches`.  (Note, this ROS package may be renamed
to `unified_launches` at some point in the future.)

The basic structure of the unified launch files is as follows:

* There is a `bin` directory that contains a bunch of common
  robot configurations (e.g. `keyboard_drive`, `keyboard_navigate`,
  etc.)  These programs are run by:

        rosrun ubiquity_launches PROGRAM_NAME

  where `PROGRAM_NAME` is the name of the program.

* Inside each of these programs is a call to common piece of code
  called `roslauncher`.  The `roslauncher` program does the following:

  * It reads the `ROS_MASTER_URI` environment variable to determine
    the DNS (Domain Naming Service) for the Robot.

  * It accesses the robot to determine a ROS package that contains
    all of the configuration information for the robot (e.g. `loki_robot`,
    `magni_robot`, `botvac_robot`, etc.)

  * It sets up X11 forwarding between the robot and the desktop/laptop
    computer you are starting the ROS launch on.  X11 is the underlying
    network window system used by Linux and it allows programs to display
    content on windows remote from the robot.

  * It starts the top level ROS launch file passing in a bunch of arguments
    that specify where everything is (e.g. the robot DNS name, the name
    of the robot configuration package, etc.)  Some of these nodes are
    started on your desktop/laptop.

* The launch files a partitioned into directories that start with `m_*
  and `n_*`.  Launch files that launch multiple ROS nodes and launch
  that start with `n_*` launch a single ROS node.

In order for unified launch files to work, there is a requirement that
your robot and your desktop/laptop both have the same modules loaded
into their respective `~/catkin_ws/src` directories.

## ROS Launch File Issues

There are several issues about ROS launch files that need to
be discussed:

* The `<arg>` tag is used heavily and needs to be fully understood.

* Launch file parameterization allows the same launch files
  to be used for different robot platforms and configurations.

### The `<arg>` Tag

Some documentation for ROS launch files can be seen by following
the hypertext links below:

* [ROS Launch Overview](http://wiki.ros.org/roslaunch) provides an
  overview of the ROS launch file architecture.

* [ROS launch XML file format](http://wiki.ros.org/roslaunch/XML)
  provides documentation of the XML format used for writing
  ROS launch files.

* [ROS launch `<arg>` tag](http://wiki.ros.org/roslaunch/XML/arg)
  is the documentation for the `<arg ... >` tag.

The `<arg>` tag is singled out because it is used in the
launch files to pass robot platform information, such as the robot type.
If you do not understand the `<arg>`, you will not understand
the launch files.

The `<arg>` tag has three forms:

* `<arg name="required" />`: This specifies a launch file input name.
  Think of this as an argument variable for routine.

* `<arg name="optional" default="value" />`: This specifies a launch
  file input name with a default value that will be used if not
  is specified at "call" time.

* `<arg name="foo" value="bar" />`: This form has two different usages.
  When at the level immediately inside of a `<launch> ... </launch>`
  pair, this form defines a convenience value that can be used to
  improve overall legibility.  Think of this as a kind of macro
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
          
          <!-- Optional arguments: -->
          <arg name="fiducial_size" default=".200" />
          
          <!-- Convenience arguments: -->
          <arg name="short value="a somewhat long string" />
          
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

The `:=` is detected and conceptually converted into an `<arg>` tag.
The line above would be represented in a launch file as:

        <include file="$(find ubiquity_launches)/n_fiducial_detect.launch">
          <arg name="robot_base" value="magni" />
        </include>

Hopefully this explanation of the `<arg>` tag is a little
more informative that the official ROS documentation.

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

### Basic File Structure:

There are two primary files in this repository -- short
shell scripts and the launch files.

The shell scripts are kept in the `bin` directory and usually
consist of only a few lines and look as follows:

        #!/usr/bin/env bash
        rosrun ubiquity_launches roslauncher m_TOP_LEVEL.launch.xml

where:

* `m_TOP_LEVEL.launch.xml` is the ROS launch file that will launch all
  of the ROS nodes needed.

The structure of a unified `*.launch.xml` file is basically as follows:

        <launch>
          <!--Summary: {One line summary of what launch file does.} -->
          <!--Overview: {A multi-line summary of what the
              launch file does. -->
          
          <!-- Required Arguments: -->
          <arg name="required_argument_name1" />
            <!--required_argument_name1: {Summary of what argument is used for.} -->
          <arg name="required_argument_name2" />
            <!--required_argument_name2: {Summary of what argument is used for.} -->
          ...
          
          <!-- Convenience Arguments: -->
          <arg name="root" value="$(find ubiquity_launches)" />
          <arg name="convenience_argument_name1" value="value1" />
          <arg name="convenience_argument_name2" value="value2" />
          ...
          
          <!-- Option arguments: -->
          <arg name="optional_argument_name1" default="default_value1" />
            <!--optional_argument_name1: {Summary of optional argument 1.} -->
          <arg name="optional_argument_name2" default="default_value2" />
            <!--optional_argument_name2: {Summary of optional argument 2.} -->
          ...

          <!-- Machine information: -->
          <machine name="..." address="..." user="..." env-loader="..." />

          <!-- Includes: -->
          <include file="...">
            <arg name="arg1" value="value1" />
            <arg name="arg2" value="value2" />
            ...
          </include>
          ...
          
          <!-- Node -->
          <node pkg="package_name" type="executable_name"
           name="ros_node_name" ... >
            <!-- Optional env, remap, rosparam, and param tags -->
          </node>
        </launch>

The structure of the `m_*.launch.xml` file differs somewhat from an
`n_*.launch.xml`.  The `m_*.launch.xml` file 

The detailed structure of an `m_*.launch.xml` file is like this:

        <launch>
          <!--Summary: {One line summary.} -->
          <!--Overview: {Multi-line overview. -->
        
          <!-- Required Arguments: -->
          <arg name="robot_dir" />
            <!--robot_dir: The robot launch files and parameters directory. -->
          <arg name="robot_host" />
            <!--robot_host: The DNS address for the robot. -->
          <arg name="robot_user" />
            <!--robot_user: The user account on the robot to use. -->
          <arg name="viewer_host" />
            <!--viewer_host: The DNS address for the viewer machine with a display. -->
          <arg name="viewer_user" default=""/>
            <!--viewer_user: The user account on the viewer computer to use. -->
        
There five arguments that are present in all `m_*.launch files`:

* `robot_dir`: This is the directory of the package that contains all of the
   robot configuration.  For the Loki robot, this directory location
   ROS `loki_robot` package (e.g. `rospack find loki_robot`.)

* `robot_host`: This is the DNS address of the robot in the form of
  `ROBOT_HOSTNAME.local`, where `ROBOT_HOSTNAME` is the host name of
  the robot.

* `robot_user`: This is the robot machine user account name (usually `ros`.)

* `remote_host`: This is the DNS address of the remote host that will
  run viewer programs (e.g. `rviz`, `rqt_image_view`, etc.) and
  control programs.

* `remote_user`: This is the remote machine user account name (usually something
  other than `ros`, like `wayne`, `rohan`, `alan`, etc.)

The convenience arguments usually contain `ul` which is the location
of the ROS `ubiquity_launches` package:

          <!-- Convenience Arguments: -->
          <arg name="ul" value="$(find ubiquity_launches)" />
          ...
        
Some `m_*.launch.xml` files have optional arguments and some do not:

          <!-- Optional arguments: -->
          <arg name="optional_name1..." default="..." />
            <!--optional_name1: {Description of optional_name1.} -->
          ...
        
The rest of an `m_*.launch.xml` file consists of `<include ...>`
directives.

The call to another `m_*.launch.xml` directory almost
always looks as follows:

          <include file="$(arg_ul)/m_MLAUNCH_NAME/launch/m_MLAUNCH_NAME.launch.xml">
            <arg name="robot_dir" value="$(arg robot_dir)" />
            <arg name="robot_host" value="$(arg robot_host)" />
            <arg name="robot_user" value="$(arg robot_user)" />
            <arg name="remote_host" value="$(arg remote_host)" />
            <arg name="remote_user" value="$(arg remote_user)" />
          </include>

where `MLAUNCH_NAME` is the base name of the `m_*.launch.xml` file that
is being included.  Notice that the 5 standard arguments thar are passed in
are passed down to the next level down `m_.launch.xml` file.

There are two flavors of `n_*.launch.xml` file depending upon whether
the node is meant to be started on the remote machine (i.e. your
desktop/laptop) or on the robot.

The XML for a remote `n_*.launch.xml` file looks as follows:

          <include file $(arg_ul)/n_NLAUNCH_NAME/launch/n_NLAUNCH_NAME.launch.xml">
            <arg name="robot_dir" value="$(arg robot_dir)" />
            <arg name="machine_name" value="remote" />
            <arg name="machine_host" value="$(arg remote_host)" />
            <arg name="machine_user" value="$(arg remote_user)" />
            <!-- Additional node specific arguments go here -->
          </include>

where `NLAUNCH_NAME` is the base name of the `n_*.launch.xml` file that
is being included.

The XML for a robot `n_*.launch.xml` file looks as follows:

          <include file $(arg_ul)/n_NLAUNCH_NAME/launch/n_NLAUNCH_NAME.launch.xml">
            <arg name="robot_dir" value="$(arg robot_dir)" />
            <arg name="machine_name" value="robot" />
            <arg name="machine_host" value="$(arg robot_host)" />
            <arg name="machine_user" value="$(arg robot_user)" />
            <!-- Additional node specific arguments go here -->
          </include>

where `NLAUNCH_NAME` is the base name of the `n_*.launch.xml` file that
is being included.

## Development Environment Details

On a 64-bit Ubuntu 14.04 computer (or vritual machine)
please do the following to set up the Ubiquity Robotics environment:

        wget https://raw.githubusercontent.com/UbiquityRobotics/ubiquity_launches/master/bin/devel_install.sh
        sh ./devel_install.sh
        source ~/.bashrc

If everything works, you should be able to run the simulator:

        rosrun ubiquity_launches keyboard_navigate

### Bringing Up a Robot

> *Wayne: Random comment.  I am making a decision here that we are
> going to change the default use and host name to `ros@robot`.
> It seems way less confusing.*

Now that you have your catkin workspace set up, we can start to
connect to your robot.  Basically, you start with a preloaded
micro-SD card that you plug into your Raspberry Pi 2.  (Remember,
we shorten 'Raspberry Pi 2' down to 'RasPi2'.)  If you do not
have preloaded micro-SD card, please see the
[micro-SD loading instructions](https://github.com/UbiquityRobotics/ubiquity_main/blob/master/Doc_Downloading_and_Installing_the_Ubiquity_Ubuntu_ROS_Kernel_Image.md).

1. Please plug an Ethernet cable between your router and your RasPi2.
   Now plug the micro-SD card into the RasPi2 and power it up.
   Wait for a minute or so and do the following:

        ping -c 5 robot.local

   If you development machine can contact the `ubuntu.local`, you will
   get 6 lines that look somewhat as follows:

        PING robot.local (192.168.1.129) 56(84) bytes of data.
        64 bytes from 192.168.1.129: icmp_seq=1 ttl=64 time=0.507 ms
        64 bytes from 192.168.1.129: icmp_seq=2 ttl=64 time=0.482 ms
        64 bytes from 192.168.1.129: icmp_seq=3 ttl=64 time=0.444 ms
        64 bytes from 192.168.1.129: icmp_seq=4 ttl=64 time=0.490 ms
        64 bytes from 192.168.1.129: icmp_seq=5 ttl=64 time=0.528 ms

   The internet address (`192.168.1.129`) will be different as will the
   ping times (i.e. `0.507`, `0.482', etc.)  If you get the above,
   you have a wired connection to your robot.

2. Now we get to use a program called `ssh` for Secure Shell,
   which allows us to connect to a shell program running on
   another machine.

   Please run the following command to connect to the `ros` account
   on the RasPi2:

        ssh ros@robot.local

    You will be prompted for a password. please enter `rosrobot` as
    the password.

    Next it will print out some stuff like:

        Welcome to Ubuntu 14.04.4 LTS (GNU/Linux 3.18.0-25-rpi2 armv7l)
        ros@robot:~$

    You are now connected to the robot.

3. The very first thing we will do is change both the host name
   and the account password.

   Under linux, each machine is given a unique name (e.g. tobor,
   my_robot, jerky, etc.) on your local network.  In addition,
   Linux supports multiple user accounts on the machine.  Each
   user account has its own password.  For the system image on
   the micro-SD card that you plugged into the Raspi2, the initial
   machine name is `robot` and the account name is `ros`.  We
   need to change `robot` to something else and we need change
   the password from `rosrobot` to something else.

   First think of a new password.  You might as well pick a pretty
   good password, since we are going to set things up so that you
   do not have to type it in very often.  A good password usually
   consists of a combination of upper and lower case letters
   along with at least one digit and one puncutation character.
   
   In order to change the password, run the following command:

        sudo passwd

   You will be prompted for the original password (e.g. `rosrobot`)

        [sudo] password for robot:

   And you will type in the new password in twice.  The password
   characters will not show on the screen as yout type them in:

        Enter new UNIX password: 
        Retype new UNIX password: 

   If you get `Sorry, passwords do not match`, please try again.

   You also need to pick a new name for the robot.  In this case
   we recommend that you specify a robot name with all lower case
   letters.  You will be typing this robot name occasionally, so
   do not make it too hard on yourself.  In the command below,
   replace `NEW_HOSTNAME` with the name you selected:

        sudo echo "NEW_HOSTNAME" > /etc/hostname
        hostname
        # The new robot host name should print out; if not try again

   Now reboot the robot:

        sudo reboot

   This will cause the robot to reboot.

4. After wait for about a minute, please repeat the steps 1 and 2
   to log into the robot using your `NEW_HOSTNAME`:

        ping -c 5 NEW_HOSTNAME.local
        ssh ubuntu@NEW_HOSTNAME.local
        Password:
        # Type in the new password.

5. We are going to set up secure shell on the robot.  There are two
   steps here.  First, we will generate public/private key pair
   for your robot. Second, we will make it so that you can run
   `ssh` between your machine and itself without prompting you
   for a password.  This is called a recursive ssh and it does
   occasionally happen.

   First, we will create the public/private key pair:

        ssh-keygen -t rsa

   Second, we will make sure that we can do the recursive call to `ssh`:

        ssh-copy-id ros@NEW_HOSTNAME.local
        # You will prompted for your new password

   Now we verify that we can do the recursive `ssh`:

        ssh ros@NEW_HOSTNAME.local

   If you are prompted for a password, something has gone wrong and you
   should restart this step from the beginning.

   If you did not get prompted for a password, you succeeded and you can
   simply type the following command to close out the `ssh` connection:

        exit

6. Now we need to log out from robot and do two more `ssh` configuration
   on your desktop/laptop.  The purpose of this configuration is to make
   for sure that `ssh` is not always prompting you for passwords.

   First, we exit the robot:

        exit

   Now we generate a public/private key pair for you laptop/desktop:

        ssh-keygen -t rsa

   Now we make sure that we can recursivly connecto to ourselves.
   Notice thae we are using accent graves (i.e. back qutoes (\`)
   rather than single quotes (') in the command below

        ssh-copy-id `whoami`@`hostname`.local

   If you get prompted for a password, provide the password for your
   desktop/laptop.

   Now verify that you can recursively `ssh` from desktop/laptop,
   to itself.

        ssh `hostname`.local

   If you are not prompted for a password, you succeeded.  If you are
   prompted for a password, something went wrong and you need to try
   the `ssh-copy-id` command again.  Type:

        exit

   To close the recursive connection.

   We are almost done.  Now we want to be able to log into your
   robot without a password.  Do the following:

        ssh-copy-id ros@NEW_HOSTNAME.local

   where `NEW_HOSTNAME` is the hostname you picked for your robot.
   If you are prompted for a password, use the new password you
   created for the robot.

   Verify that you can login into the robot without a password.

        ssh ros@NEW_HOSTNAME.local

   As usual, you do not want to be promted for a password.  Type
   the following command to disconnect.

        exit

That covers the initial setup for now.

### Initial setup:

We are going to assume that you are doing all of these instructions
from the comfort of your own development machine.  We further
assume that you can connect to the `ubuntu` user on your robot.
We further assume that you can already access the robot via
secure shell.

1. Install some software on your development machine:

        sudo apt-get install -y ssh fuse sshfs
        sudo gpasswd -a $USER fuse

2. Let's make sure that you can log into your robot via `ssh`.
   Please replace ROBOT with your robot host name:

        ssh ubuntu@ROBOT.local echo hello
        # You will probably be prompted for the `ubuntu` account
        # password (usually `ubuntu`).  When done it will print
        # out `hello`.


## Notes

What follows are some notes that the people at Ubiquty Robots need
to remind them of what to do....


### Creating the Catkin Work Spaces

We will create two identical catkin work spaces on both
the development processor and the robot processor.  We
assume that ROS has already been installed.


1. On the development machine:

        mkdir -p ~/catkin_ws/src

2. Now we want to populate the work space with 4 ROS packages:

        cd ~/cakin_ws/src
        git clone https://github.com/UbiquityRobotics/ubiquity_launches.git
        git clone https://github.com/UbiquityRobotics/raspicam_node.git
        git clone https://github.com/UbiquityRobotics/userland.git
        git clone https://github.com/UbiquityRobotics/robot_upstart.git
        source ~/.bashrc

   {Wayne: Make sure $ROS_PACKAGE_PATH is correct.}

3. Now we initialize the Catkin work space:

        (cd ~/catkin_ws ; cakin_make)

4. Now we create `~/.ros_setup` to have the following content
   using your favorite editor (e.g. `nano ~/.ros_setup`):

        # Only `source /opt/ros/indigo/setup.bash` if we have not already done so.
        # We assume that this script does not change very often:
        if [ -d "/opt/ros/indigo/bin" ] ; then
            case ":$PATH:" in
            *:/opt/ros/indigo/bin:*) ;;
            *) source /opt/ros/indigo/setup.bash ;;
            esac
        fi

        # Only `source ~/catkin_ws/devel/setup` if it exists:
        if [ -f $HOME/catkin_ws/devel/setup.sh ] ; then
            source $HOME/catkin_ws/devel/setup.bash ;
        fi

        export ROS_CATKIN_WS=$HOME/catkin_ws
        export ROS_HOSTNAME=`hostname`.local
        export ROS_MASTER_URI=http://`hostname`.local:11311
        export ROSLAUNCH_SSH_UNKNOWN=1

5. Now we edit `~/.profile` to add the following lines:

        # Always set up the ROS envionment:
        . $HOME/.ros_setup

   right after the line:

        #umask 022

6. Now we add the following lines to the end of `~/.bashrc`:

        # Setup ROS environment:
        source ~/.ros_setup

        # Uncomment one of these to select a robot:
        #export ROS_MASTER_URI=http://ROBOT.local:11311       # Robot
        #export ROS_MASTER_URI=http://`hostname`.local:11311  # Simulator

    Where `ROBOT` is the host name of the robot.  Remove the `#` comment
    character in front of the `ROS_MASTER_URI` you want to use.  You can
    only select one at a time.  Now run the folowing command:

        source ~/.bashrc

7. Now repeat these step over on the robot.

### X11 Forwarding


In order to enable X11 forwarding via secure shell, we need
to edit the file `/etc/ssh/ssh_config` on both the development
processor and the robot processor.

1. {Wayne: This step appears to be unnecessary.}

   Edit `/etc/ssh/ssh_config` on the devolopment machine:

        sudo nano /etc/ssh/ssh_config

   Find the two lines:

        #   ForwardX11 yes
        #   ForwardX11Trusted yes

   and remove the `[#]` characters to make them look as follows:

        #   ForwardX11 yes
        #   ForwardX11Trusted yes

   Write the contents out using `[Ctr-O]` followed by exiting
   using `[Ctrl-X]` keys.

2. Verify that X11 forwarding is working on the development machine.
   Please notice the `-Y` option:

        ssh -Y ubuntu@ROBOT.local xterm

   An `xterm` window should pop up on the development processor
   display.  The prompt inside the xterm should say `USER@ROBOT:~`.
   With your mouse, click inside  `xterm` window and type:

        hostname

   inside `xterm` window followed by the `[Enter]` key.  The response
   should be `ROBOT`.  Things are working.  Again, in the `xterm`
   window type:

        exit

   and now the `xterm` window will go away.        

3. Now we do the same edits over on the robot processor.  Type
   the following being sure to specify the `-Y` option:
 

        ssh -Y ubuntu@ROBOT.local
        sudo nano /etc/ssh/ssh_config

   Remove the `#` characters in front of `ForwardX11` and 
   `ForwardX11Trusted`.  Write everything out with `[Ctrl-O]`
   followed by `[Ctrl-X]`.  Please stay logged onto the
   robot processor:

4. Testing that we got it write is done as follows:

        ssh -Y ubuntu@ROBOT.local xterm

   will cause an xterm to pop up on the development processor.
   (Just for your information, the X11 protocol is being forwarded
   through two secure shell X11 protocol forwarders, since there
   no display on the robot processor.)  Dismiss the `xterm` window
   by typing `exit` in the `xterm` window.  Finally, exit from the
   robot processor by typing:

        exit

That takes care of X11 forwarding issues.

### Using `sshfs`

The `sshfs` program is used to mount the `src` directory from
the robot processor over on to the development processor

To mount the robot `~/catkin_ws/src` on top of the local `~/catkin_ws/src`:

        sshfs -o nonempty -o idmap=user ROBOT.local:/home/`whoami`/catkin_ws/src ~/catkin_ws/src 

To unmount:

        fusermount -u ~/catkin_ws/src

### Making `roscore` Run All the Time

*{Wayne: make `n_forever` match `n_sleep_forever`.}*

This is done using the following command:

        ssh ROBOT.local
        rosrun robot_upstart install --user=USER ubiquity_launches/n_sleep_forever/launch/n_sleep_forever.launch
        sudo service ubiquity start

### Install the `pudb` Python Debugger:

Please do the following:

        sudo apt-get install -y python python-pip xterm
        sudo pip install pudb


### Run `raspicam_view`

Please do the following on the development machine:

        rosrun ubiquity_launches raspicam_view

When `rqt_image_view` pops up please select the
`/n_raspicam/camera/image/compressed` topic.  When you are
done, please type `[Ctrl-C]`.

## Launch Files


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

