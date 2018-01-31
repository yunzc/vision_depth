# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/yun/vision/vision_depth

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yun/vision/vision_depth

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/yun/vision/vision_depth/CMakeFiles /home/yun/vision/vision_depth/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/yun/vision/vision_depth/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named image_depth

# Build rule for target.
image_depth: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 image_depth
.PHONY : image_depth

# fast build rule for target.
image_depth/fast:
	$(MAKE) -f CMakeFiles/image_depth.dir/build.make CMakeFiles/image_depth.dir/build
.PHONY : image_depth/fast

#=============================================================================
# Target rules for targets named image_segmentation

# Build rule for target.
image_segmentation: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 image_segmentation
.PHONY : image_segmentation

# fast build rule for target.
image_segmentation/fast:
	$(MAKE) -f CMakeFiles/image_segmentation.dir/build.make CMakeFiles/image_segmentation.dir/build
.PHONY : image_segmentation/fast

#=============================================================================
# Target rules for targets named test_opencv

# Build rule for target.
test_opencv: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test_opencv
.PHONY : test_opencv

# fast build rule for target.
test_opencv/fast:
	$(MAKE) -f CMakeFiles/test_opencv.dir/build.make CMakeFiles/test_opencv.dir/build
.PHONY : test_opencv/fast

#=============================================================================
# Target rules for targets named feature_match

# Build rule for target.
feature_match: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 feature_match
.PHONY : feature_match

# fast build rule for target.
feature_match/fast:
	$(MAKE) -f CMakeFiles/feature_match.dir/build.make CMakeFiles/feature_match.dir/build
.PHONY : feature_match/fast

#=============================================================================
# Target rules for targets named sift_img_depth

# Build rule for target.
sift_img_depth: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sift_img_depth
.PHONY : sift_img_depth

# fast build rule for target.
sift_img_depth/fast:
	$(MAKE) -f CMakeFiles/sift_img_depth.dir/build.make CMakeFiles/sift_img_depth.dir/build
.PHONY : sift_img_depth/fast

#=============================================================================
# Target rules for targets named downscale

# Build rule for target.
downscale: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 downscale
.PHONY : downscale

# fast build rule for target.
downscale/fast:
	$(MAKE) -f CMakeFiles/downscale.dir/build.make CMakeFiles/downscale.dir/build
.PHONY : downscale/fast

#=============================================================================
# Target rules for targets named match_test

# Build rule for target.
match_test: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 match_test
.PHONY : match_test

# fast build rule for target.
match_test/fast:
	$(MAKE) -f CMakeFiles/match_test.dir/build.make CMakeFiles/match_test.dir/build
.PHONY : match_test/fast

#=============================================================================
# Target rules for targets named camera_calibration

# Build rule for target.
camera_calibration: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 camera_calibration
.PHONY : camera_calibration

# fast build rule for target.
camera_calibration/fast:
	$(MAKE) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/build
.PHONY : camera_calibration/fast

camera_calibration.o: camera_calibration.cpp.o

.PHONY : camera_calibration.o

# target to build an object file
camera_calibration.cpp.o:
	$(MAKE) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.o
.PHONY : camera_calibration.cpp.o

camera_calibration.i: camera_calibration.cpp.i

.PHONY : camera_calibration.i

# target to preprocess a source file
camera_calibration.cpp.i:
	$(MAKE) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.i
.PHONY : camera_calibration.cpp.i

camera_calibration.s: camera_calibration.cpp.s

.PHONY : camera_calibration.s

# target to generate assembly for a file
camera_calibration.cpp.s:
	$(MAKE) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.s
.PHONY : camera_calibration.cpp.s

downscale.o: downscale.cpp.o

.PHONY : downscale.o

# target to build an object file
downscale.cpp.o:
	$(MAKE) -f CMakeFiles/downscale.dir/build.make CMakeFiles/downscale.dir/downscale.cpp.o
.PHONY : downscale.cpp.o

downscale.i: downscale.cpp.i

.PHONY : downscale.i

# target to preprocess a source file
downscale.cpp.i:
	$(MAKE) -f CMakeFiles/downscale.dir/build.make CMakeFiles/downscale.dir/downscale.cpp.i
.PHONY : downscale.cpp.i

downscale.s: downscale.cpp.s

.PHONY : downscale.s

# target to generate assembly for a file
downscale.cpp.s:
	$(MAKE) -f CMakeFiles/downscale.dir/build.make CMakeFiles/downscale.dir/downscale.cpp.s
.PHONY : downscale.cpp.s

feature_match.o: feature_match.cpp.o

.PHONY : feature_match.o

# target to build an object file
feature_match.cpp.o:
	$(MAKE) -f CMakeFiles/feature_match.dir/build.make CMakeFiles/feature_match.dir/feature_match.cpp.o
.PHONY : feature_match.cpp.o

feature_match.i: feature_match.cpp.i

.PHONY : feature_match.i

# target to preprocess a source file
feature_match.cpp.i:
	$(MAKE) -f CMakeFiles/feature_match.dir/build.make CMakeFiles/feature_match.dir/feature_match.cpp.i
.PHONY : feature_match.cpp.i

feature_match.s: feature_match.cpp.s

.PHONY : feature_match.s

# target to generate assembly for a file
feature_match.cpp.s:
	$(MAKE) -f CMakeFiles/feature_match.dir/build.make CMakeFiles/feature_match.dir/feature_match.cpp.s
.PHONY : feature_match.cpp.s

image_depth.o: image_depth.cpp.o

.PHONY : image_depth.o

# target to build an object file
image_depth.cpp.o:
	$(MAKE) -f CMakeFiles/image_depth.dir/build.make CMakeFiles/image_depth.dir/image_depth.cpp.o
.PHONY : image_depth.cpp.o

image_depth.i: image_depth.cpp.i

.PHONY : image_depth.i

# target to preprocess a source file
image_depth.cpp.i:
	$(MAKE) -f CMakeFiles/image_depth.dir/build.make CMakeFiles/image_depth.dir/image_depth.cpp.i
.PHONY : image_depth.cpp.i

image_depth.s: image_depth.cpp.s

.PHONY : image_depth.s

# target to generate assembly for a file
image_depth.cpp.s:
	$(MAKE) -f CMakeFiles/image_depth.dir/build.make CMakeFiles/image_depth.dir/image_depth.cpp.s
.PHONY : image_depth.cpp.s

image_segmentation.o: image_segmentation.cpp.o

.PHONY : image_segmentation.o

# target to build an object file
image_segmentation.cpp.o:
	$(MAKE) -f CMakeFiles/image_segmentation.dir/build.make CMakeFiles/image_segmentation.dir/image_segmentation.cpp.o
.PHONY : image_segmentation.cpp.o

image_segmentation.i: image_segmentation.cpp.i

.PHONY : image_segmentation.i

# target to preprocess a source file
image_segmentation.cpp.i:
	$(MAKE) -f CMakeFiles/image_segmentation.dir/build.make CMakeFiles/image_segmentation.dir/image_segmentation.cpp.i
.PHONY : image_segmentation.cpp.i

image_segmentation.s: image_segmentation.cpp.s

.PHONY : image_segmentation.s

# target to generate assembly for a file
image_segmentation.cpp.s:
	$(MAKE) -f CMakeFiles/image_segmentation.dir/build.make CMakeFiles/image_segmentation.dir/image_segmentation.cpp.s
.PHONY : image_segmentation.cpp.s

match_test.o: match_test.cpp.o

.PHONY : match_test.o

# target to build an object file
match_test.cpp.o:
	$(MAKE) -f CMakeFiles/match_test.dir/build.make CMakeFiles/match_test.dir/match_test.cpp.o
.PHONY : match_test.cpp.o

match_test.i: match_test.cpp.i

.PHONY : match_test.i

# target to preprocess a source file
match_test.cpp.i:
	$(MAKE) -f CMakeFiles/match_test.dir/build.make CMakeFiles/match_test.dir/match_test.cpp.i
.PHONY : match_test.cpp.i

match_test.s: match_test.cpp.s

.PHONY : match_test.s

# target to generate assembly for a file
match_test.cpp.s:
	$(MAKE) -f CMakeFiles/match_test.dir/build.make CMakeFiles/match_test.dir/match_test.cpp.s
.PHONY : match_test.cpp.s

sift_img_depth.o: sift_img_depth.cpp.o

.PHONY : sift_img_depth.o

# target to build an object file
sift_img_depth.cpp.o:
	$(MAKE) -f CMakeFiles/sift_img_depth.dir/build.make CMakeFiles/sift_img_depth.dir/sift_img_depth.cpp.o
.PHONY : sift_img_depth.cpp.o

sift_img_depth.i: sift_img_depth.cpp.i

.PHONY : sift_img_depth.i

# target to preprocess a source file
sift_img_depth.cpp.i:
	$(MAKE) -f CMakeFiles/sift_img_depth.dir/build.make CMakeFiles/sift_img_depth.dir/sift_img_depth.cpp.i
.PHONY : sift_img_depth.cpp.i

sift_img_depth.s: sift_img_depth.cpp.s

.PHONY : sift_img_depth.s

# target to generate assembly for a file
sift_img_depth.cpp.s:
	$(MAKE) -f CMakeFiles/sift_img_depth.dir/build.make CMakeFiles/sift_img_depth.dir/sift_img_depth.cpp.s
.PHONY : sift_img_depth.cpp.s

test_opencv.o: test_opencv.cpp.o

.PHONY : test_opencv.o

# target to build an object file
test_opencv.cpp.o:
	$(MAKE) -f CMakeFiles/test_opencv.dir/build.make CMakeFiles/test_opencv.dir/test_opencv.cpp.o
.PHONY : test_opencv.cpp.o

test_opencv.i: test_opencv.cpp.i

.PHONY : test_opencv.i

# target to preprocess a source file
test_opencv.cpp.i:
	$(MAKE) -f CMakeFiles/test_opencv.dir/build.make CMakeFiles/test_opencv.dir/test_opencv.cpp.i
.PHONY : test_opencv.cpp.i

test_opencv.s: test_opencv.cpp.s

.PHONY : test_opencv.s

# target to generate assembly for a file
test_opencv.cpp.s:
	$(MAKE) -f CMakeFiles/test_opencv.dir/build.make CMakeFiles/test_opencv.dir/test_opencv.cpp.s
.PHONY : test_opencv.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... image_depth"
	@echo "... image_segmentation"
	@echo "... test_opencv"
	@echo "... feature_match"
	@echo "... sift_img_depth"
	@echo "... downscale"
	@echo "... match_test"
	@echo "... camera_calibration"
	@echo "... camera_calibration.o"
	@echo "... camera_calibration.i"
	@echo "... camera_calibration.s"
	@echo "... downscale.o"
	@echo "... downscale.i"
	@echo "... downscale.s"
	@echo "... feature_match.o"
	@echo "... feature_match.i"
	@echo "... feature_match.s"
	@echo "... image_depth.o"
	@echo "... image_depth.i"
	@echo "... image_depth.s"
	@echo "... image_segmentation.o"
	@echo "... image_segmentation.i"
	@echo "... image_segmentation.s"
	@echo "... match_test.o"
	@echo "... match_test.i"
	@echo "... match_test.s"
	@echo "... sift_img_depth.o"
	@echo "... sift_img_depth.i"
	@echo "... sift_img_depth.s"
	@echo "... test_opencv.o"
	@echo "... test_opencv.i"
	@echo "... test_opencv.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

