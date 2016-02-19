#!/usr/bin/env python

import yaml
import sys

def main():
    argv = sys.argv
    base_file_names = argv[1:]
    for base_file_name in base_file_names:
	yaml_file_name = base_file_name + ".yaml"
	launch_file_name = base_file_name + ".launch.xml"
	
	with open(yaml_file_name, "r") as yaml_file:
	    yaml_data = yaml_file.read()
	    yaml_dict = yaml.load(yaml_data)
	    print("yaml_dict:", yaml_dict)
	    is_robot = yaml_dict["machine"] == "robot"

	    topics = []
	    if "topics" in yaml_dict:
		topics = yaml_dict["topics"]

	    with open(launch_file_name, "w") as launch_file:
		launch_file.write("<launch>\n")

		launch_file.write("  <!-- Required arguments -->\n")
		launch_file.write("  <arg name=\"robot_platform\" />\n")
		launch_file.write("    <!--robot_platform: The robot platform (e.g. 'loki'.) -->\n")
		if is_robot:
		    launch_file.write("  <arg name=\"robot_host\" />\n")
		    launch_file.write("    <!--robot_host: The robot host address. -->\n")
		launch_file.write("  <arg name=\"robot_user\" />\n")
		launch_file.write("    <!--robot_host: The robot user name. -->\n")
		launch_file.write("\n")

		launch_file.write("  <!-- Optional arguments -->\n")
		for topic in topics:
		    launch_file.write("  <arg name=\"{0}_topic\" default=\"{1}\" />\n".
		      format(topic, topic))
		    launch_file.write("    <!--{0}: more here -->\n".format(topic))
		launch_file.write("\n")

		if is_robot:
		    launch_file.write("  <!-- Robot machine declaration -->\n")
		    launch_file.write("  <machine name=\"robot\"\n")
		    launch_file.write("   address=\"$(arg robot_host)\"\n")
		    launch_file.write("   user=\"$(arg robot_user)\"\n")
		    launch_file.write("   env-loader=\"/tmp/env_loader.sh\" />\n")
		    launch_file.write("\n")

		# Write out the <node ... >:
		launch_file.write("  <node name=\"{0}\"\n".format(yaml_dict["name"]))
		if is_robot:
		    launch_file.write("   machine=\"robot\"\n")
		launch_file.write("   pkg=\"{0}\"\n".format(yaml_dict["package"]))
		launch_file.write("   type=\"{0}\" >\n".format(yaml_dict["bin"]))

		for topic in topics:
		    launch_file.write("    <remap from=\"{0}\" to=\"$(arg {0}_topic)\" />\n".
		      format(topic))

		launch_file.write("  </node>\n")
		launch_file.write("</launch>\n")

		

if __name__ == "__main__":
    main()
