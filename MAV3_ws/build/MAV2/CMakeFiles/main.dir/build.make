# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build

# Include any dependencies generated for this target.
include MAV2/CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include MAV2/CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include MAV2/CMakeFiles/main.dir/flags.make

MAV2/CMakeFiles/main.dir/node/main.cpp.o: MAV2/CMakeFiles/main.dir/flags.make
MAV2/CMakeFiles/main.dir/node/main.cpp.o: /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/node/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MAV2/CMakeFiles/main.dir/node/main.cpp.o"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/node/main.cpp.o -c /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/node/main.cpp

MAV2/CMakeFiles/main.dir/node/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/node/main.cpp.i"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/node/main.cpp > CMakeFiles/main.dir/node/main.cpp.i

MAV2/CMakeFiles/main.dir/node/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/node/main.cpp.s"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/node/main.cpp -o CMakeFiles/main.dir/node/main.cpp.s

MAV2/CMakeFiles/main.dir/src/takeoff.cpp.o: MAV2/CMakeFiles/main.dir/flags.make
MAV2/CMakeFiles/main.dir/src/takeoff.cpp.o: /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/src/takeoff.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object MAV2/CMakeFiles/main.dir/src/takeoff.cpp.o"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/takeoff.cpp.o -c /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/src/takeoff.cpp

MAV2/CMakeFiles/main.dir/src/takeoff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/takeoff.cpp.i"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/src/takeoff.cpp > CMakeFiles/main.dir/src/takeoff.cpp.i

MAV2/CMakeFiles/main.dir/src/takeoff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/takeoff.cpp.s"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2/src/takeoff.cpp -o CMakeFiles/main.dir/src/takeoff.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/node/main.cpp.o" \
"CMakeFiles/main.dir/src/takeoff.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: MAV2/CMakeFiles/main.dir/node/main.cpp.o
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: MAV2/CMakeFiles/main.dir/src/takeoff.cpp.o
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: MAV2/CMakeFiles/main.dir/build.make
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/liborocos-kdl.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/liborocos-kdl.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libtf2_ros.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libactionlib.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libmessage_filters.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libroscpp.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libpthread.a
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/librosconsole.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libtf2.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/librostime.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /opt/ros/noetic/lib/libcpp_common.so
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main: MAV2/CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main"
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MAV2/CMakeFiles/main.dir/build: /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/devel/lib/mav3/main

.PHONY : MAV2/CMakeFiles/main.dir/build

MAV2/CMakeFiles/main.dir/clean:
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 && $(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : MAV2/CMakeFiles/main.dir/clean

MAV2/CMakeFiles/main.dir/depend:
	cd /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/src/MAV2 /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2 /home/lee/workspace/ros_ws/Synchronize_takeoff/MAV3_ws/build/MAV2/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MAV2/CMakeFiles/main.dir/depend

