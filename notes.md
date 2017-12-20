# Ubiquity Launches Notes

This is a summary of some conversations between Wayne, Rohan, and Jim about
what to do with launch files in Ubiquity.

## Issues with the Current System

There is general agreement that the current Ubiquity Launches architecture has
some issues:

* It over uses the remote launch facilities of `roslaunch`.

* The launch files are all collected in one directory, rather than having
  a launch file associted with each Node.

* The launch files are brittle, so simple syntatic errors cause difficult
  to detect errors.

* The concept of using `sshfs` to unifiy the `catkin_ws` directory on both
  the robot and workstation caused problems.

* The system was an "all or nothing" architecture.  It was not possible to
  incrementally switch over to it.

## Goals for Replacement System

These are the general goals for a replacement system:

* Support incremental development where we can slowy nudge the launch system forward.
  No "all or nothing" architecture.  We want this task to be a small number of reasonable
  tasks that take several hours each to implement.

* Use `sshfs` in a less fragile way.

* Use existing technology like `systemd` to manage some of the start-up dependencies.

## Some Initial Thoughts

* Try to make it so that the users are not fighting against the launch files.

* Do not add any more environment variables.  ROS already has too many.

* Rather than running one launch file to launch nodes on both the robot and workstation,
  there will be launch files that are run on the robot, and launch files that are run
  on the workstation.  The robot launch files should work without requiring the workstation
  launch files to be running.  The workstation launch files will generally run RViz or
  ROS image_view.  Thus the user will have two (or more) terminal windows open, one
  that is ssh'ed into the robot and another that is on the workstation.

* Use `systemd` to start and stop `roscore`.  Maybe use it to start and stop
  other facilities as well.

* Provide a way for the "application" to figure out if everything is healthy.
  This can be done by adding ROS diagonostic messages to RaspiCam, the Magni Node,
  the Loki Node, etc.  For example, the Fidicial Follow demo will verify that
  both a wheel base (i.e. Magni or Loki) and RaspiCam are working.  If not, an
  error message is generated.

* Make the wheel controller launch file be platform neutral.  This means it will run
  either the Magni_Node or the Loki_Node.

* This was not discussed, but we should make it easy to get nodes to launch inside
  of a debugger.  Using X forwarding, a node can come up in either a Python debugger
  or a C++ debugger.

* Launch files should be stored with the node.



