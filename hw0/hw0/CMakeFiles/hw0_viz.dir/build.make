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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.14.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.14.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/rohunkulkarni/CS225A/apps/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/rohunkulkarni/CS225A/apps/cs225a/hw0

# Include any dependencies generated for this target.
include hw0/CMakeFiles/hw0_viz.dir/depend.make

# Include the progress variables for this target.
include hw0/CMakeFiles/hw0_viz.dir/progress.make

# Include the compile flags for this target's objects.
include hw0/CMakeFiles/hw0_viz.dir/flags.make

hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o: hw0/CMakeFiles/hw0_viz.dir/flags.make
hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o: hw0_viz.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/rohunkulkarni/CS225A/apps/cs225a/hw0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o"
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o -c /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0_viz.cpp

hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw0_viz.dir/hw0_viz.cpp.i"
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0_viz.cpp > CMakeFiles/hw0_viz.dir/hw0_viz.cpp.i

hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw0_viz.dir/hw0_viz.cpp.s"
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0_viz.cpp -o CMakeFiles/hw0_viz.dir/hw0_viz.cpp.s

# Object files for target hw0_viz
hw0_viz_OBJECTS = \
"CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o"

# External object files for target hw0_viz
hw0_viz_EXTERNAL_OBJECTS =

../bin/hw0/hw0_viz: hw0/CMakeFiles/hw0_viz.dir/hw0_viz.cpp.o
../bin/hw0/hw0_viz: hw0/CMakeFiles/hw0_viz.dir/build.make
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-common/build/libsai2-common.a
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/chai3d/build/libchai3d.a
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0_viz: /usr/local/lib/libtinyxml2.dylib
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-model/build/libsai2-model.a
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0_viz: /usr/local/lib/libtinyxml2.dylib
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-model/rbdl/build/librbdl.dylib
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-graphics/build/libsai2-graphics.a
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0_viz: /usr/local/lib/libtinyxml2.dylib
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/chai3d/build/libchai3d.a
../bin/hw0/hw0_viz: /usr/local/lib/libjsoncpp.dylib
../bin/hw0/hw0_viz: /usr/local/lib/libhiredis.dylib
../bin/hw0/hw0_viz: /usr/local/lib/libglfw.dylib
../bin/hw0/hw0_viz: /Users/rohunkulkarni/CS225A/core/sai2-model/rbdl/build/librbdl.dylib
../bin/hw0/hw0_viz: /usr/local/lib/libjsoncpp.dylib
../bin/hw0/hw0_viz: /usr/local/lib/libhiredis.dylib
../bin/hw0/hw0_viz: /usr/local/lib/libglfw.dylib
../bin/hw0/hw0_viz: hw0/CMakeFiles/hw0_viz.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/rohunkulkarni/CS225A/apps/cs225a/hw0/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/hw0/hw0_viz"
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hw0_viz.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hw0/CMakeFiles/hw0_viz.dir/build: ../bin/hw0/hw0_viz

.PHONY : hw0/CMakeFiles/hw0_viz.dir/build

hw0/CMakeFiles/hw0_viz.dir/clean:
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 && $(CMAKE_COMMAND) -P CMakeFiles/hw0_viz.dir/cmake_clean.cmake
.PHONY : hw0/CMakeFiles/hw0_viz.dir/clean

hw0/CMakeFiles/hw0_viz.dir/depend:
	cd /Users/rohunkulkarni/CS225A/apps/cs225a/hw0 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/rohunkulkarni/CS225A/apps/cs225a /Users/rohunkulkarni/CS225A/apps/cs225a/hw0 /Users/rohunkulkarni/CS225A/apps/cs225a/hw0 /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0 /Users/rohunkulkarni/CS225A/apps/cs225a/hw0/hw0/CMakeFiles/hw0_viz.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hw0/CMakeFiles/hw0_viz.dir/depend

