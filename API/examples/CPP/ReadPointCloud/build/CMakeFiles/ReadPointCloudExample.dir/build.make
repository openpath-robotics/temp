# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jh/buffing/API/examples/CPP/ReadPointCloud

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jh/buffing/API/examples/CPP/ReadPointCloud/build

# Include any dependencies generated for this target.
include CMakeFiles/ReadPointCloudExample.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ReadPointCloudExample.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ReadPointCloudExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ReadPointCloudExample.dir/flags.make

CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o: CMakeFiles/ReadPointCloudExample.dir/flags.make
CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o: ../ReadPointCloudExample.cpp
CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o: CMakeFiles/ReadPointCloudExample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jh/buffing/API/examples/CPP/ReadPointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o -MF CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o.d -o CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o -c /home/jh/buffing/API/examples/CPP/ReadPointCloud/ReadPointCloudExample.cpp

CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jh/buffing/API/examples/CPP/ReadPointCloud/ReadPointCloudExample.cpp > CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.i

CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jh/buffing/API/examples/CPP/ReadPointCloud/ReadPointCloudExample.cpp -o CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.s

# Object files for target ReadPointCloudExample
ReadPointCloudExample_OBJECTS = \
"CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o"

# External object files for target ReadPointCloudExample
ReadPointCloudExample_EXTERNAL_OBJECTS =

ReadPointCloudExample_Release: CMakeFiles/ReadPointCloudExample.dir/ReadPointCloudExample.cpp.o
ReadPointCloudExample_Release: CMakeFiles/ReadPointCloudExample.dir/build.make
ReadPointCloudExample_Release: /opt/Photoneo/PhoXiControl-1.13.4/API/lib/libPhoXi_API_gcc9.3.0_Release.so
ReadPointCloudExample_Release: CMakeFiles/ReadPointCloudExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jh/buffing/API/examples/CPP/ReadPointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ReadPointCloudExample_Release"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ReadPointCloudExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ReadPointCloudExample.dir/build: ReadPointCloudExample_Release
.PHONY : CMakeFiles/ReadPointCloudExample.dir/build

CMakeFiles/ReadPointCloudExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ReadPointCloudExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ReadPointCloudExample.dir/clean

CMakeFiles/ReadPointCloudExample.dir/depend:
	cd /home/jh/buffing/API/examples/CPP/ReadPointCloud/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jh/buffing/API/examples/CPP/ReadPointCloud /home/jh/buffing/API/examples/CPP/ReadPointCloud /home/jh/buffing/API/examples/CPP/ReadPointCloud/build /home/jh/buffing/API/examples/CPP/ReadPointCloud/build /home/jh/buffing/API/examples/CPP/ReadPointCloud/build/CMakeFiles/ReadPointCloudExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ReadPointCloudExample.dir/depend

