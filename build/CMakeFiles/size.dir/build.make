# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vicenterey/esp/projects_tf/model/person_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vicenterey/esp/projects_tf/model/person_detection/build

# Utility rule file for size.

# Include any custom commands dependencies for this target.
include CMakeFiles/size.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/size.dir/progress.make

CMakeFiles/size: person_detection.map
	/usr/bin/cmake -D "IDF_SIZE_TOOL=/home/vicenterey/esp/idf-tools/python_env/idf5.3_py3.10_env/bin/python;-m;esp_idf_size" -D MAP_FILE=/home/vicenterey/esp/projects_tf/model/person_detection/build/person_detection.map -D OUTPUT_JSON= -P /home/vicenterey/esp/idf/esp-idf/tools/cmake/run_size_tool.cmake

size: CMakeFiles/size
size: CMakeFiles/size.dir/build.make
.PHONY : size

# Rule to build all files generated by this target.
CMakeFiles/size.dir/build: size
.PHONY : CMakeFiles/size.dir/build

CMakeFiles/size.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/size.dir/cmake_clean.cmake
.PHONY : CMakeFiles/size.dir/clean

CMakeFiles/size.dir/depend:
	cd /home/vicenterey/esp/projects_tf/model/person_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vicenterey/esp/projects_tf/model/person_detection /home/vicenterey/esp/projects_tf/model/person_detection /home/vicenterey/esp/projects_tf/model/person_detection/build /home/vicenterey/esp/projects_tf/model/person_detection/build /home/vicenterey/esp/projects_tf/model/person_detection/build/CMakeFiles/size.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/size.dir/depend
