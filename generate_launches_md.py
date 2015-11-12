#!/usr/bin/env python
# Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

# Import some libraries:
import glob
import os
import re
import xml.etree.ElementTree as ET

def main():
    """ main program """

    full_file_names = []
    for root, directory_names, file_names in os.walk("."):
	#print("root={0}, directory_names={1}, file_names={2}".
	#  format(root, directory_names, file_names))

	for file_name in file_names:
	    if file_name.endswith(".launch.xml") or \
	      file_name.endswith(".launch"):
		full_file_name = os.path.join(root, file_name)
		full_file_names.append(full_file_name)
		#print("full_file_name={0}".format(full_file_name))

    # Make sure we do everything in sorted order:
    full_file_names.sort()

    # Now process each 
    launch_files = []
    for full_file_name in full_file_names:
	launch_files.append(Launch_File.file_parse(full_file_name))

    # Open the markdown file:
    md_file = open("launches.md", "wa")
    md_file.write("# Ubiquity Launches\n\n")

    # Write out each section:
    md_file.write("The following launch file directories are available:\n\n")
    for launch_file in launch_files:
	launch_file.summary_write(md_file)

    # Write out each section:
    for launch_file in launch_files:
	launch_file.section_write(md_file)

    # Close the markdown file:
    md_file.close()

class Launch_File:
    def __init__(self, name, argument_comments, requireds, optionals, macros):
	""" *Launch_File*: Initialize the *Launch_File* object (i.e. *self*)
	    with *name*, *argument_comments*, *requireds*, *optionals*, and
	    *macros*.
	"""

	# Verify argument types:
	assert isinstance(name, str)
	assert isinstance(argument_comments, dict)
	assert isinstance(requireds, list)
	assert isinstance(optionals, list)
	assert isinstance(macros, dict)

	# Load up *self*:
	self.name = name
	self.argument_comments = argument_comments
	self.requireds = requireds
	self.optionals = optionals
	self.macros = macros

    @staticmethod
    def file_parse(full_file_name):
	""" *Launch_File*: Process one launch file.
	"""

	# Verify argument types:
	assert isinstance(full_file_name, str)

	# Do some file/directory stuff:
	root_path = full_file_name.split('/')
	assert len(root_path) >= 1, \
	  "Root path '{0}' is too short".format(root_path)
	launch_file_name = root_path[1]

	# Read in *full_file_name*:
	xml_file = open(full_file_name, "ra")
	xml_text = xml_file.read()
	xml_file.close()

	# Find all comments in *xml_text* (*re.DOTALL* allows regular
	# expressons to span multiple lines):
	comments = re.findall("<!--.*?-->", xml_text, re.DOTALL)

	# Now we war interested in comments of the form "<!--NAME:...-->"
	# where "NAME" is an alphanumeric identifier.  When we find such
	# a comment, we stuff the comment contents into *argument_comments*
	# keyed by "NAME":
	argument_comments = {}
	for comment in comments:
	    # Strip the "<!--" and "-->" off of *comment*:
	    comment = comment[4:-3]
	    #print("    comment1: '{0}'".format(comment))

	    if not comment.startswith(' '):
		# Grab *argument_name*:
		colon_index = comment.find(':')
		if colon_index >= 0:
		    argument_name = comment[:colon_index]
		    comment = comment[colon_index + 1:]
		    #print("    comment2: '{0}'".format(comment))

		    # Now reformat multiple lines so that they all are indented
		    # by *prefix*:
		    prefix = "  "
		    if argument_name == "Overview":
			prefix = ""
		    lines = comment.split('\n')
		    for index in range(len(lines)):
			lines[index] = prefix + lines[index].strip()
			#print("line='{0}'".format(line))
		    comment = '\n'.join(lines)
		    #print("    comment3: '{0}'".format(comment))

		    # Stuff the resulting *comment* into *argument_names*:
		    argument_comments[argument_name] = comment
		    #print("    comment4: '{0}'".format(comment))

	# Parse the XML:
	tree = ET.fromstring(xml_text)
	requireds = []
	optionals = []
	macros = {}

	# Visit all of the tags under the <Launch> tag:
	for child in tree:
	    # We only care about <Arg ...> tags:
	    if child.tag == "arg":
		attributes = child.attrib
		name = attributes["name"]
		if "default" in attributes:
		    # We have an optional argument:
		    optionals.append(child)
		elif "value" in attributes:
		    # We have a convenience argument (i.e. macro):
		    macros[name] = attributes["value"]
		else:
		    # We have a required argument:
		    requireds.append(child)
 
	launch_file = Launch_File(launch_file_name,
	   argument_comments, requireds, optionals, macros)
	return launch_file
 
    def summary_write(self, md_file):
	""" *Launch_File*: Write out the summary item for the *Launch_File*
	    object (i.e. *self*) to *md_file*:
	"""

	# Verify argument types:
	assert isinstance(md_file, file)

	# Grab some values from *self*:
	name = self.name
	argument_comments = self.argument_comments

	# Write out an item:
	if "Summary" in argument_comments:
	    md_file.write("* {0}:\n{1}\n".
	      format(name, argument_comments["Summary"]))
	else:
	    md_file.write("* {0}: (No Summary Available)\n".format(name))
	md_file.write("\n")

    def section_write(self, md_file):
	""" *Launch_File*: Write out the section for the *Launch_File* object
	    (i.e. *self*) to *md_file*.
	"""
		
	# Verify argument types:
	assert isinstance(md_file, file)

	# Grab some values from *self*:
	name = self.name
	argument_comments = self.argument_comments
	requireds = self.requireds
	optionals = self.optionals
	macros = self.macros

	# Output the section heading:
	md_file.write("## {0} Launch File Directory\n\n".format(name))

	# Output the overview comment:
	if "Overview" in argument_comments:
	    #print("Overview:{0}".format(argument_comments["Overview"]))
	    overview_comments = argument_comments["Overview"]
	    md_file.write("{0}\n\n".format(overview_comments))

	# Output each *requried* and *optional* argument:
	arguments_count = len(requireds) + len(optionals)
	if arguments_count == 0:
	    md_file.write("This launch file has no arguments.\n\n")
	elif arguments_count == 1:
	    md_file.write("This launch file has the following argument:\n\n")
	else:
	    md_file.write("This launch file has the following arguments:\n\n")

	# Output each *required* argument:
	for required in requireds:
	    attributes = required.attrib
	    name = attributes["name"]
	    md_file.write("* {0} (Required):\n".format(name))
	    if name in argument_comments:
		md_file.write("{0}\n".format(argument_comments[name]))
	    md_file.write("\n")
	    
	# Output each *optional* argument:
	for optional in optionals:
	    attributes = optional.attrib
	    name = attributes["name"]
	    default = attributes["default"]
	    md_file.write("* {0} (Optional, default: '{1}'):\n".
	      format(name, default))
	    if name in argument_comments:
		md_file.write("{0}\n".format(argument_comments[name]))
	    md_file.write("\n")

if __name__ == "__main__":
    main()

