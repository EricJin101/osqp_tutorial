# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_SOURCE_DIR = /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build

# Include any dependencies generated for this target.
include CMakeFiles/osqp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/osqp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/osqp.dir/flags.make

CMakeFiles/osqp.dir/src/main.cc.o: CMakeFiles/osqp.dir/flags.make
CMakeFiles/osqp.dir/src/main.cc.o: ../src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/osqp.dir/src/main.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osqp.dir/src/main.cc.o -c /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/src/main.cc

CMakeFiles/osqp.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osqp.dir/src/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/src/main.cc > CMakeFiles/osqp.dir/src/main.cc.i

CMakeFiles/osqp.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osqp.dir/src/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/src/main.cc -o CMakeFiles/osqp.dir/src/main.cc.s

# Object files for target osqp
osqp_OBJECTS = \
"CMakeFiles/osqp.dir/src/main.cc.o"

# External object files for target osqp
osqp_EXTERNAL_OBJECTS =

osqp: CMakeFiles/osqp.dir/src/main.cc.o
osqp: CMakeFiles/osqp.dir/build.make
osqp: /usr/local/lib/libOsqpEigen.so.0.6.3
osqp: /usr/local/lib/libosqp.so
osqp: CMakeFiles/osqp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable osqp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/osqp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/osqp.dir/build: osqp

.PHONY : CMakeFiles/osqp.dir/build

CMakeFiles/osqp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/osqp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/osqp.dir/clean

CMakeFiles/osqp.dir/depend:
	cd /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build /home/trunk/jinxinyu_project/private/osqp_tutorial/c_plus/osqp-eigen/build/CMakeFiles/osqp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/osqp.dir/depend

