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
CMAKE_SOURCE_DIR = /home/jonathan/Desktop/5G_uav_sim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build

# Include any dependencies generated for this target.
include lib/common_cpp/CMakeFiles/unit_tests.dir/depend.make

# Include the progress variables for this target.
include lib/common_cpp/CMakeFiles/unit_tests.dir/progress.make

# Include the compile flags for this target's objects.
include lib/common_cpp/CMakeFiles/unit_tests.dir/flags.make

lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o: lib/common_cpp/CMakeFiles/unit_tests.dir/flags.make
lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o: ../../lib/common_cpp/src/unit_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/common_cpp/src/unit_tests.cpp

lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/unit_tests.dir/src/unit_tests.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/common_cpp/src/unit_tests.cpp > CMakeFiles/unit_tests.dir/src/unit_tests.cpp.i

lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/unit_tests.dir/src/unit_tests.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/common_cpp/src/unit_tests.cpp -o CMakeFiles/unit_tests.dir/src/unit_tests.cpp.s

# Object files for target unit_tests
unit_tests_OBJECTS = \
"CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o"

# External object files for target unit_tests
unit_tests_EXTERNAL_OBJECTS =

lib/common_cpp/unit_tests: lib/common_cpp/CMakeFiles/unit_tests.dir/src/unit_tests.cpp.o
lib/common_cpp/unit_tests: lib/common_cpp/CMakeFiles/unit_tests.dir/build.make
lib/common_cpp/unit_tests: lib/common_cpp/CMakeFiles/unit_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable unit_tests"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/unit_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/common_cpp/CMakeFiles/unit_tests.dir/build: lib/common_cpp/unit_tests

.PHONY : lib/common_cpp/CMakeFiles/unit_tests.dir/build

lib/common_cpp/CMakeFiles/unit_tests.dir/clean:
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp && $(CMAKE_COMMAND) -P CMakeFiles/unit_tests.dir/cmake_clean.cmake
.PHONY : lib/common_cpp/CMakeFiles/unit_tests.dir/clean

lib/common_cpp/CMakeFiles/unit_tests.dir/depend:
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jonathan/Desktop/5G_uav_sim /home/jonathan/Desktop/5G_uav_sim/lib/common_cpp /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/common_cpp/CMakeFiles/unit_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/common_cpp/CMakeFiles/unit_tests.dir/depend

