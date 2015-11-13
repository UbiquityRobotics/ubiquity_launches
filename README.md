# ubiquity_launches.

This repository contains a ROS launch and configuration files
for the Ubiquity Robots platforms.

It adheres to the
[Robot Launch Repository Standard](https://github.com/UbiquityRobotics/ubiquity_main/blob/master/Doc_Robot_Launch_Repositories.md)
document.

This repository is broken into some categories:

* `bin`: The `bin` directory contains a bunch of executables shell
  scripts that fire off a launch file.

* `n_*`: The `n_*` directories contain the launch files and
  configuration files needed to launch a single ROS Node.

* `m_*`: The `m_*` directories will launch Multiple ROS nodes.

* `rviz_*: The `rviz_*` directories are used to launch the RViz
  program configured to view a corresponding robot program.

* others: Other directories contain miscellaneous launch files.

The actual documentation is scraped from the various files
and put into a single
[`launches.md`](launches.md) file.
