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
CMAKE_SOURCE_DIR = /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build

# Include any dependencies generated for this target.
include CMakeFiles/FullAPIExample.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FullAPIExample.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FullAPIExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FullAPIExample.dir/flags.make

CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o: CMakeFiles/FullAPIExample.dir/flags.make
CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o: ../FullAPIExample.cpp
CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o: CMakeFiles/FullAPIExample.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o -MF CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o.d -o CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o -c /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/FullAPIExample.cpp

CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/FullAPIExample.cpp > CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.i

CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/FullAPIExample.cpp -o CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.s

# Object files for target FullAPIExample
FullAPIExample_OBJECTS = \
"CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o"

# External object files for target FullAPIExample
FullAPIExample_EXTERNAL_OBJECTS =

FullAPIExample: CMakeFiles/FullAPIExample.dir/FullAPIExample.cpp.o
FullAPIExample: CMakeFiles/FullAPIExample.dir/build.make
FullAPIExample: /opt/Photoneo/PhoXiControl-1.13.4/API/lib/libPhoXi_API_gcc9.3.0_Release.so
FullAPIExample: CMakeFiles/FullAPIExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FullAPIExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FullAPIExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FullAPIExample.dir/build: FullAPIExample
.PHONY : CMakeFiles/FullAPIExample.dir/build

CMakeFiles/FullAPIExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FullAPIExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FullAPIExample.dir/clean

CMakeFiles/FullAPIExample.dir/depend:
	cd /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build /opt/Photoneo/PhoXiControl-1.13.4/API/examples/CPP/FullAPI/build/CMakeFiles/FullAPIExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FullAPIExample.dir/depend
