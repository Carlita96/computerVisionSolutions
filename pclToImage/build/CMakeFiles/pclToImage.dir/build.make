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
CMAKE_SOURCE_DIR = /home/carla/Documents/softwareSolutions/pclToImage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/carla/Documents/softwareSolutions/pclToImage/build

# Include any dependencies generated for this target.
include CMakeFiles/pclToImage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pclToImage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pclToImage.dir/flags.make

CMakeFiles/pclToImage.dir/pclToImage.cpp.o: CMakeFiles/pclToImage.dir/flags.make
CMakeFiles/pclToImage.dir/pclToImage.cpp.o: ../pclToImage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/carla/Documents/softwareSolutions/pclToImage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pclToImage.dir/pclToImage.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pclToImage.dir/pclToImage.cpp.o -c /home/carla/Documents/softwareSolutions/pclToImage/pclToImage.cpp

CMakeFiles/pclToImage.dir/pclToImage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pclToImage.dir/pclToImage.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/carla/Documents/softwareSolutions/pclToImage/pclToImage.cpp > CMakeFiles/pclToImage.dir/pclToImage.cpp.i

CMakeFiles/pclToImage.dir/pclToImage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pclToImage.dir/pclToImage.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/carla/Documents/softwareSolutions/pclToImage/pclToImage.cpp -o CMakeFiles/pclToImage.dir/pclToImage.cpp.s

# Object files for target pclToImage
pclToImage_OBJECTS = \
"CMakeFiles/pclToImage.dir/pclToImage.cpp.o"

# External object files for target pclToImage
pclToImage_EXTERNAL_OBJECTS =

pclToImage: CMakeFiles/pclToImage.dir/pclToImage.cpp.o
pclToImage: CMakeFiles/pclToImage.dir/build.make
pclToImage: /usr/local/lib/libpcl_surface.so
pclToImage: /usr/local/lib/libpcl_keypoints.so
pclToImage: /usr/local/lib/libpcl_tracking.so
pclToImage: /usr/local/lib/libpcl_recognition.so
pclToImage: /usr/local/lib/libpcl_stereo.so
pclToImage: /usr/local/lib/libpcl_outofcore.so
pclToImage: /usr/local/lib/libpcl_people.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_system.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pclToImage: /usr/lib/x86_64-linux-gnu/libboost_regex.so
pclToImage: /usr/lib/x86_64-linux-gnu/libqhull.so
pclToImage: /usr/lib/libOpenNI.so
pclToImage: /usr/lib/libOpenNI2.so
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libfreetype.so
pclToImage: /usr/lib/x86_64-linux-gnu/libz.so
pclToImage: /usr/lib/x86_64-linux-gnu/libjpeg.so
pclToImage: /usr/lib/x86_64-linux-gnu/libpng.so
pclToImage: /usr/lib/x86_64-linux-gnu/libtiff.so
pclToImage: /usr/lib/x86_64-linux-gnu/libexpat.so
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
pclToImage: /usr/local/lib/libpcl_registration.so
pclToImage: /usr/local/lib/libpcl_segmentation.so
pclToImage: /usr/local/lib/libpcl_features.so
pclToImage: /usr/local/lib/libpcl_filters.so
pclToImage: /usr/local/lib/libpcl_sample_consensus.so
pclToImage: /usr/local/lib/libpcl_ml.so
pclToImage: /usr/local/lib/libpcl_visualization.so
pclToImage: /usr/local/lib/libpcl_search.so
pclToImage: /usr/local/lib/libpcl_kdtree.so
pclToImage: /usr/local/lib/libpcl_io.so
pclToImage: /usr/local/lib/libpcl_octree.so
pclToImage: /usr/local/lib/libpcl_common.so
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libfreetype.so
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
pclToImage: /usr/lib/x86_64-linux-gnu/libz.so
pclToImage: /usr/lib/x86_64-linux-gnu/libGLEW.so
pclToImage: /usr/lib/x86_64-linux-gnu/libSM.so
pclToImage: /usr/lib/x86_64-linux-gnu/libICE.so
pclToImage: /usr/lib/x86_64-linux-gnu/libX11.so
pclToImage: /usr/lib/x86_64-linux-gnu/libXext.so
pclToImage: /usr/lib/x86_64-linux-gnu/libXt.so
pclToImage: CMakeFiles/pclToImage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/carla/Documents/softwareSolutions/pclToImage/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pclToImage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pclToImage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pclToImage.dir/build: pclToImage

.PHONY : CMakeFiles/pclToImage.dir/build

CMakeFiles/pclToImage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pclToImage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pclToImage.dir/clean

CMakeFiles/pclToImage.dir/depend:
	cd /home/carla/Documents/softwareSolutions/pclToImage/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/carla/Documents/softwareSolutions/pclToImage /home/carla/Documents/softwareSolutions/pclToImage /home/carla/Documents/softwareSolutions/pclToImage/build /home/carla/Documents/softwareSolutions/pclToImage/build /home/carla/Documents/softwareSolutions/pclToImage/build/CMakeFiles/pclToImage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pclToImage.dir/depend
