#!/usr/bin/env python
# Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

# Import some libraries:
import glob
import os
import re
import xml.etree.ElementTree as ET

def main():
    """ main program """

    # Search for launch file names:
    launch_file_names = []
    for root, directory_names, file_names in os.walk("."):
	#print("root={0}, directory_names={1}, file_names={2}".
	#  format(root, directory_names, file_names))

	for file_name in file_names:
	    full_file_name = os.path.join(root, file_name)
	    if file_name.endswith(".launch.xml") or \
	      file_name.endswith(".launch"):
		launch_file_names.append(full_file_name)
		#print("launch_file_name={0}".format(launch_file_name))

    # Search for executable files in `bin` directory:
    executable_file_names = []
    for root, directory_names, file_names in os.walk("./bin"):
	for file_name in file_names:
	    full_file_name = os.path.join(root, file_name)
	    if os.access(full_file_name, os.X_OK) and \
	      os.path.isfile(full_file_name) and \
	      not full_file_name.endswith("~"):
		executable_file_names.append(full_file_name)
		#print("executable: {0}".format(full_file_name))

    # Make sure we do everything in sorted order:
    launch_file_names.sort()
    executable_file_names.sort()

    # Now process each *execubale_file_name*:
    executable_files = []
    for executable_file_name in executable_file_names:
	executable_file = Executable_File.file_parse(executable_file_name)
	executable_files.append(executable_file)

    # Now process each *launch_file_name*:
    launch_files = []
    for launch_file_name in launch_file_names:
	launch_files.append(Launch_File.file_parse(launch_file_name))

    # Open the markdown file:
    md_file = open("launches.md", "wa")
    md_file.write("# Ubiquity Launches\n\n")

    # Write out each executable file summary:
    md_file.write("The following executables are available in `bin`:\n\n")
    for executable_file in executable_files:
	executable_file.summary_write(md_file)

    # Write out each launch file summary:
    md_file.write("The following launch file directories are available:\n\n")
    for launch_file in launch_files:
	launch_file.summary_write(md_file)

    # Write out each executable file section:
    md_file.write("## Executables\n\n")
    for executable_file in executable_files:
	executable_file.section_write(md_file)

    # Write out each launch file section:
    md_file.write("## Launch File Directories\n\n")
    for launch_file in launch_files:
	launch_file.section_write(md_file)

    # Close the markdown file:
    md_file.close()

    # Create *launch_files_table*:
    launch_files_table = {}
    for launch_file in launch_files:
	launch_files_table[launch_file.name] = launch_file

    # Recursively visit each *executable_file*:
    for executable_file in executable_files:
	executable_file.visit(launch_files_table)

    # Print out each *launch_file* that was not visited:
    for launch_file in launch_files:
	if not launch_file.visited:
	    print("Launch File: '{0}' is unused".format(launch_file.name))

def macro_replace(match, macros):
    """ Replace "$(arg VALUE)", with the value from *macros* and return it.
    """

    # Verify arguments:
    #assert isinstance(match, re.MatchObject)
    assert isinstance(macros, dict)

    # We ASSUME that the format of the string is "$(COMMAND VALUE)".
    # First split the match string at the space:
    splits = match.group().split()

    # Grabe the *command* and *value*:
    command = splits[0][2:]
    value = splits[1][:-1]

    # Now only substitute argument values:
    result = ""
    #print("macros=", macros)
    if command == "arg" and value in macros:
	# We have an argument value:
	result = macros[value]
    else:
	# Leave everything else more or less alone:
	result = "[{0}:{1}]".format(command, value)
    return result

class Executable_File:
    """ *Executable_File* is a class that represents a executable file.
    """

    def __init__(self, name, summary, overview_lines, launch_base_name):
	""" *Executable_File*: ...
	"""

	# Verify argument types:
	assert isinstance(name, str)
	assert isinstance(summary, str)
	assert isinstance(overview_lines, list)
	assert launch_base_name == None or isinstance(launch_base_name, str)

	# Load up *self*:
	self.name = name
	self.summary = summary
	self.overview_lines = overview_lines
	self.launch_base_name = launch_base_name

    @staticmethod
    def file_parse(full_file_name):
	""" *Executable_File*: Parse *full_file_name* and scrape out
	    the usable documentation.
	"""

	# Verify argument types:
	assert isinstance(full_file_name, str)
	splits = full_file_name.split('/')
	executable_name = splits[2]
	
	in_file = open(full_file_name, "ra")
	lines = in_file.readlines()
	in_file.close()

	launch_base_name = None
	summary = ""
	overview_lines = []
	for line in lines:
	    if line.startswith("##"):
		comment_line = line[2:].strip()
		if comment_line.startswith("Summary:"):
		    # We have a summary:
		    summary = comment_line[8:].strip()
		elif comment_line.startswith("Overview:"):
		    pass
		else:
		    overview_lines.append(comment_line)
	    if line.startswith("roslaunch"):
		splits = line.split(' ')
		launch_file_name = splits[2]
		#print("lauch_file_name={0}".format(launch_file_name))
		splits = launch_file_name.split('.')
		launch_base_name = splits[0]

	return Executable_File(
	  executable_name, summary, overview_lines, launch_base_name)

    def section_write(self, md_file):
	""" *Executable_File*: Write the secton for the *Executable_File*
	    object (i.e. *self*) out to *md_file*.
	"""

	# Verify argument types:
	assert isinstance(md_file, file)

	# Grab some values from *self*:
	name = self.name
	overview_lines = self.overview_lines

	# Write out the executable section:
	md_file.write("### `{0}` Executable:\n\n".format(name))
	for overview_line in overview_lines:
	    md_file.write("{0}\n".format(overview_line))
	md_file.write("\n")

    def summary_write(self, md_file):
	""" *Executable_File*: Write the summary for the *Executable_File*
	    object (i.e. *self*) out to *md_file*.
	"""

	# Verify argument types:
	assert isinstance(md_file, file)

	# Grab some values from *self*:
	name = self.name
	summary = self.summary

	# Write out the *summary*:
	md_file.write("* `{0}`: {1}\n\n".format(name, summary))

    def visit(self, launch_files_table):
	""" *Executable_File*: Recursively visit the *Launch_File* object
	    referenced by the *Executable_File* object (i.e. *self*).
	"""

	# Verify argument types:
	assert isinstance(launch_files_table, dict)

	launch_base_name = self.launch_base_name
	if launch_base_name in launch_files_table:
	    launch_file = launch_files_table[launch_base_name]
	    launch_file.visit(launch_files_table)
	elif launch_base_name != None:
	    print("Executable '{0}' can not find launch file directory '{1}'".
	      format(self.name, launch_base_name))

class Launch_File:
    def __init__(self, name,
      argument_comments, requireds, optionals, macros, includes, conditionals):
	""" *Launch_File*: Initialize the *Launch_File* object (i.e. *self*)
	    with *name*, *argument_comments*, *requireds*, *optionals*,
	    *macros*, *includes*, and *conditionals*.
	"""

	# Verify argument types:
	assert isinstance(name, str)
	assert isinstance(argument_comments, dict)
	assert isinstance(requireds, list)
	assert isinstance(optionals, list)
	assert isinstance(macros, dict)
	assert isinstance(includes, list)
	assert isinstance(conditionals, list)

	# Load up *self*:
	self.name = name
	self.argument_comments = argument_comments
	self.requireds = requireds
	self.optionals = optionals
	self.macros = macros
	self.includes = includes
	self.conditionals = conditionals
	self.visited = False

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
	#print("{0}:".format(full_file_name))
	tree = ET.fromstring(xml_text)
	requireds = []
	optionals = []

	# Create an empty *macros* mapping table:
	macros = {}
	includes = []
	conditionals = []

	# Visit all of the tags under the <Launch> tag:
	for child in tree:
	    # We only care about <Arg ...> tags:
	    child_tag = child.tag
            attributes = child.attrib
	    if child_tag == "arg":
		# We have <arg ...>:
		name = attributes["name"]
		if "default" in attributes:
		    # We have an optional argument:
		    optionals.append(child)
		elif "value" in attributes:
		    # We have a convenience argument (i.e. macro):
		    value = attributes["value"]
		    macros[name] = value
		    #print("macros['{0}'] = '{1}'".format(name, value))
		else:
		    # We have a required argument:
		    requireds.append(child)
	    elif child_tag == "include":
		# We have <include ...>:
		if "file" in attributes:
		    # Repeatably perform macro substitution on *file_name*:
		    file_name = attributes["file"]
		    file_name_previous = ""
		    while file_name_previous != file_name:
			file_name_previous = file_name
			file_name = re.sub(r"\$\(arg .*?\)",
			  lambda match: macro_replace(match, macros), file_name)
			#print("'{0}'=>'{1}'".
			#  format(file_name_previous, file_name))

		    # Determine if we have a `robot_base` argument to deal with:
		    if file_name.find("[arg:robot_base]") >= 0:
			# We have a `robot_base` argument:

			#print("robot_base <include...>: {0}".
			#  format(file_after))

			# Search for each *robot_base*:
			for robot_base in ["loki", "magni"]:
			    # Subsitute in *robot_base*:
			    #print("robot_base='{0}'".format(robot_base))
			    #print("file_name before='{0}'".format(file_name))
			    conditional_file_name = \
			      re.sub(r"(\[arg:robot_base\])",
			      robot_base, file_name)
			    #print("file_name after='{0}'".
			    #  format(conditional_file_name))

                            # Extract the *conditional_base_name*:
			    splits = conditional_file_name.split('/')
			    include_xml_name = splits[-1]
			    splits = include_xml_name.split('.')
			    conditional_base_name = splits[0]
			    #print("condtional_base_name='{0}:{1}'\n".
			    #  format(conditional_base_name, robot_base))

			    # Keep track of *conditional_base_name*
			    # in *conditionals* list:
			    conditionals.append(conditional_base_name)
                    else:
			# Now grab the *include_base_name*:
			splits = file_name.split('/')
			include_xml_name = splits[-1]
			splits = include_xml_name.split('.')
			include_base_name = splits[0]
			#print("include_base_name='{0}'".
			#  format(include_base_name))

			# Collect *include_base_name* in *includes* list:
			includes.append(include_base_name)

	# Construct and return *launch_file*:
	launch_file = Launch_File(launch_file_name, argument_comments,
	  requireds, optionals, macros, includes, conditionals)
	return launch_file
 
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
	md_file.write("### `{0}` Launch File Directory\n\n".format(name))

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

    def show(self):
	""" *Launch_File*: Show contents of the *Launch_File* object
	    (i.e. *self*).
	"""

	# Show *self*:
        print("Name: {0}".format(self.name))
	for include in self.includes:
	    print("  Include: {0}".format(include))

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
	    md_file.write("* `{0}`:\n{1}\n".
	      format(name, argument_comments["Summary"]))
	else:
	    md_file.write("* {0}: (No Summary Available)\n".format(name))
	md_file.write("\n")

    def visit(self, launch_files_table):
	""" *Launch_File*: Recursively visit the *Launch_File* object
	    (i.e. *self*) using *launch_files_table*.
	"""
	
	# Verify argument types:
	assert isinstance(launch_files_table, dict)

	if not self.visited:
	    self.visited = True

	    for include in self.includes:
		if include in launch_files_table:
		    child = launch_files_table[include]
                    child.visit(launch_files_table)
		else:
		    print("Launch file '{0}' references non-existant '{1}'".
		      format(self.name, include))

	    for conditional in self.conditionals:
		#print("conditional:{0}".format(conditional))
		if conditional in launch_files_table:
		    child = launch_files_table[conditional]
		    child.visit(launch_files_table)
		else:
		    print("Launch file '{0}' uses non-existant base file '{1}'".
		      format(self.name, conditional))
	
if __name__ == "__main__":
    main()

