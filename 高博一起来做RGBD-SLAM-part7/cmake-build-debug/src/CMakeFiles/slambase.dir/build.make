# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/yk/桌面/clion-2018.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/yk/桌面/clion-2018.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug"

# Include any dependencies generated for this target.
include src/CMakeFiles/slambase.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/slambase.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/slambase.dir/flags.make

src/CMakeFiles/slambase.dir/slamBase.cpp.o: src/CMakeFiles/slambase.dir/flags.make
src/CMakeFiles/slambase.dir/slamBase.cpp.o: ../src/slamBase.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/slambase.dir/slamBase.cpp.o"
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slambase.dir/slamBase.cpp.o -c "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/src/slamBase.cpp"

src/CMakeFiles/slambase.dir/slamBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slambase.dir/slamBase.cpp.i"
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/src/slamBase.cpp" > CMakeFiles/slambase.dir/slamBase.cpp.i

src/CMakeFiles/slambase.dir/slamBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slambase.dir/slamBase.cpp.s"
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/src/slamBase.cpp" -o CMakeFiles/slambase.dir/slamBase.cpp.s

# Object files for target slambase
slambase_OBJECTS = \
"CMakeFiles/slambase.dir/slamBase.cpp.o"

# External object files for target slambase
slambase_EXTERNAL_OBJECTS =

../lib/libslambase.a: src/CMakeFiles/slambase.dir/slamBase.cpp.o
../lib/libslambase.a: src/CMakeFiles/slambase.dir/build.make
../lib/libslambase.a: src/CMakeFiles/slambase.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../lib/libslambase.a"
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && $(CMAKE_COMMAND) -P CMakeFiles/slambase.dir/cmake_clean_target.cmake
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slambase.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/slambase.dir/build: ../lib/libslambase.a

.PHONY : src/CMakeFiles/slambase.dir/build

src/CMakeFiles/slambase.dir/clean:
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" && $(CMAKE_COMMAND) -P CMakeFiles/slambase.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/slambase.dir/clean

src/CMakeFiles/slambase.dir/depend:
	cd "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII" "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/src" "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug" "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src" "/home/yk/桌面/高翔一起来做RGBD-SLAM/rgbd-slam-tutorial-gx/part VII/cmake-build-debug/src/CMakeFiles/slambase.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : src/CMakeFiles/slambase.dir/depend

