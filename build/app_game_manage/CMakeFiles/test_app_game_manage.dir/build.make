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
CMAKE_SOURCE_DIR = /home/yyy/Desktop/dbz/force_master_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yyy/Desktop/dbz/force_master_ws/build

# Include any dependencies generated for this target.
include app_game_manage/CMakeFiles/test_app_game_manage.dir/depend.make

# Include the progress variables for this target.
include app_game_manage/CMakeFiles/test_app_game_manage.dir/progress.make

# Include the compile flags for this target's objects.
include app_game_manage/CMakeFiles/test_app_game_manage.dir/flags.make

app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.o: app_game_manage/CMakeFiles/test_app_game_manage.dir/flags.make
app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.o: /home/yyy/Desktop/dbz/force_master_ws/src/app_game_manage/test/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyy/Desktop/dbz/force_master_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.o"
	cd /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_app_game_manage.dir/test/main.cpp.o -c /home/yyy/Desktop/dbz/force_master_ws/src/app_game_manage/test/main.cpp

app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_app_game_manage.dir/test/main.cpp.i"
	cd /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyy/Desktop/dbz/force_master_ws/src/app_game_manage/test/main.cpp > CMakeFiles/test_app_game_manage.dir/test/main.cpp.i

app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_app_game_manage.dir/test/main.cpp.s"
	cd /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyy/Desktop/dbz/force_master_ws/src/app_game_manage/test/main.cpp -o CMakeFiles/test_app_game_manage.dir/test/main.cpp.s

# Object files for target test_app_game_manage
test_app_game_manage_OBJECTS = \
"CMakeFiles/test_app_game_manage.dir/test/main.cpp.o"

# External object files for target test_app_game_manage
test_app_game_manage_EXTERNAL_OBJECTS =

/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: app_game_manage/CMakeFiles/test_app_game_manage.dir/test/main.cpp.o
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: app_game_manage/CMakeFiles/test_app_game_manage.dir/build.make
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libroscpp.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librostime.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libcpp_common.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/local/protobuf/lib/libprotobuf.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/local/protobuf/lib/libprotoc.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libapp_game_manage.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libprotocol.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libur_driver.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libtcp_client.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libforce_control.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libalgorithm.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/local/protobuf/lib/libprotobuf.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/local/protobuf/lib/libprotoc.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libdevice_manage.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libthread_manager.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/libudp_client.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libroscpp.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/librostime.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /opt/ros/noetic/lib/libcpp_common.so
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage: app_game_manage/CMakeFiles/test_app_game_manage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yyy/Desktop/dbz/force_master_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage"
	cd /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_app_game_manage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app_game_manage/CMakeFiles/test_app_game_manage.dir/build: /home/yyy/Desktop/dbz/force_master_ws/devel/lib/app_game_manage/test_app_game_manage

.PHONY : app_game_manage/CMakeFiles/test_app_game_manage.dir/build

app_game_manage/CMakeFiles/test_app_game_manage.dir/clean:
	cd /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage && $(CMAKE_COMMAND) -P CMakeFiles/test_app_game_manage.dir/cmake_clean.cmake
.PHONY : app_game_manage/CMakeFiles/test_app_game_manage.dir/clean

app_game_manage/CMakeFiles/test_app_game_manage.dir/depend:
	cd /home/yyy/Desktop/dbz/force_master_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yyy/Desktop/dbz/force_master_ws/src /home/yyy/Desktop/dbz/force_master_ws/src/app_game_manage /home/yyy/Desktop/dbz/force_master_ws/build /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage /home/yyy/Desktop/dbz/force_master_ws/build/app_game_manage/CMakeFiles/test_app_game_manage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app_game_manage/CMakeFiles/test_app_game_manage.dir/depend

