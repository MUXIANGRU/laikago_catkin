# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mxr/catkin_laikago/src/lcm-1.4.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mxr/catkin_laikago/src/lcm-1.4.0/build

# Include any dependencies generated for this target.
include liblcm-test/CMakeFiles/lcm-buftest-sender.dir/depend.make

# Include the progress variables for this target.
include liblcm-test/CMakeFiles/lcm-buftest-sender.dir/progress.make

# Include the compile flags for this target's objects.
include liblcm-test/CMakeFiles/lcm-buftest-sender.dir/flags.make

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/flags.make
liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o: ../liblcm-test/buftest-sender.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o"
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o   -c /home/mxr/catkin_laikago/src/lcm-1.4.0/liblcm-test/buftest-sender.c

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.i"
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/mxr/catkin_laikago/src/lcm-1.4.0/liblcm-test/buftest-sender.c > CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.i

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.s"
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/mxr/catkin_laikago/src/lcm-1.4.0/liblcm-test/buftest-sender.c -o CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.s

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.requires:

.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.requires

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.provides: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.requires
	$(MAKE) -f liblcm-test/CMakeFiles/lcm-buftest-sender.dir/build.make liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.provides.build
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.provides

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.provides.build: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o


# Object files for target lcm-buftest-sender
lcm__buftest__sender_OBJECTS = \
"CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o"

# External object files for target lcm-buftest-sender
lcm__buftest__sender_EXTERNAL_OBJECTS =

liblcm-test/lcm-buftest-sender: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o
liblcm-test/lcm-buftest-sender: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/build.make
liblcm-test/lcm-buftest-sender: lcm/liblcm.so.1.4.0
liblcm-test/lcm-buftest-sender: /usr/lib/x86_64-linux-gnu/libglib-2.0.so
liblcm-test/lcm-buftest-sender: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable lcm-buftest-sender"
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lcm-buftest-sender.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
liblcm-test/CMakeFiles/lcm-buftest-sender.dir/build: liblcm-test/lcm-buftest-sender

.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/build

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/requires: liblcm-test/CMakeFiles/lcm-buftest-sender.dir/buftest-sender.c.o.requires

.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/requires

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/clean:
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test && $(CMAKE_COMMAND) -P CMakeFiles/lcm-buftest-sender.dir/cmake_clean.cmake
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/clean

liblcm-test/CMakeFiles/lcm-buftest-sender.dir/depend:
	cd /home/mxr/catkin_laikago/src/lcm-1.4.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mxr/catkin_laikago/src/lcm-1.4.0 /home/mxr/catkin_laikago/src/lcm-1.4.0/liblcm-test /home/mxr/catkin_laikago/src/lcm-1.4.0/build /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test /home/mxr/catkin_laikago/src/lcm-1.4.0/build/liblcm-test/CMakeFiles/lcm-buftest-sender.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-sender.dir/depend

