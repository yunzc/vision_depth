# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/yun/vision/vision_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yun/vision/vision_slam

# Include any dependencies generated for this target.
include CMakeFiles/canny_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/canny_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/canny_test.dir/flags.make

CMakeFiles/canny_test.dir/canny.cpp.o: CMakeFiles/canny_test.dir/flags.make
CMakeFiles/canny_test.dir/canny.cpp.o: canny.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yun/vision/vision_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/canny_test.dir/canny.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/canny_test.dir/canny.cpp.o -c /home/yun/vision/vision_slam/canny.cpp

CMakeFiles/canny_test.dir/canny.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/canny_test.dir/canny.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yun/vision/vision_slam/canny.cpp > CMakeFiles/canny_test.dir/canny.cpp.i

CMakeFiles/canny_test.dir/canny.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/canny_test.dir/canny.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yun/vision/vision_slam/canny.cpp -o CMakeFiles/canny_test.dir/canny.cpp.s

CMakeFiles/canny_test.dir/canny.cpp.o.requires:

.PHONY : CMakeFiles/canny_test.dir/canny.cpp.o.requires

CMakeFiles/canny_test.dir/canny.cpp.o.provides: CMakeFiles/canny_test.dir/canny.cpp.o.requires
	$(MAKE) -f CMakeFiles/canny_test.dir/build.make CMakeFiles/canny_test.dir/canny.cpp.o.provides.build
.PHONY : CMakeFiles/canny_test.dir/canny.cpp.o.provides

CMakeFiles/canny_test.dir/canny.cpp.o.provides.build: CMakeFiles/canny_test.dir/canny.cpp.o


# Object files for target canny_test
canny_test_OBJECTS = \
"CMakeFiles/canny_test.dir/canny.cpp.o"

# External object files for target canny_test
canny_test_EXTERNAL_OBJECTS =

canny_test: CMakeFiles/canny_test.dir/canny.cpp.o
canny_test: CMakeFiles/canny_test.dir/build.make
canny_test: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_superres3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_face3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_img_hash3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_reg3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_shape3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_photo3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_viz3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_video3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_plot3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_text3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_flann3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_ml3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.3.1
canny_test: /opt/ros/kinetic/lib/libopencv_core3.so.3.3.1
canny_test: CMakeFiles/canny_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yun/vision/vision_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable canny_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/canny_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/canny_test.dir/build: canny_test

.PHONY : CMakeFiles/canny_test.dir/build

CMakeFiles/canny_test.dir/requires: CMakeFiles/canny_test.dir/canny.cpp.o.requires

.PHONY : CMakeFiles/canny_test.dir/requires

CMakeFiles/canny_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/canny_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/canny_test.dir/clean

CMakeFiles/canny_test.dir/depend:
	cd /home/yun/vision/vision_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yun/vision/vision_slam /home/yun/vision/vision_slam /home/yun/vision/vision_slam /home/yun/vision/vision_slam /home/yun/vision/vision_slam/CMakeFiles/canny_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/canny_test.dir/depend
