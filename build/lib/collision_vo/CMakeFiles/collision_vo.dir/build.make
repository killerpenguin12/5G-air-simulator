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
include lib/collision_vo/CMakeFiles/collision_vo.dir/depend.make

# Include the progress variables for this target.
include lib/collision_vo/CMakeFiles/collision_vo.dir/progress.make

# Include the compile flags for this target's objects.
include lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make

lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o: lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make
lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o: ../../lib/collision_vo/src/collision_vo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/collision_vo.cpp

lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_vo.dir/src/collision_vo.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/collision_vo.cpp > CMakeFiles/collision_vo.dir/src/collision_vo.cpp.i

lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_vo.dir/src/collision_vo.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/collision_vo.cpp -o CMakeFiles/collision_vo.dir/src/collision_vo.cpp.s

lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.o: lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make
lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.o: ../../lib/collision_vo/src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_vo.dir/src/utils.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/utils.cpp

lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_vo.dir/src/utils.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/utils.cpp > CMakeFiles/collision_vo.dir/src/utils.cpp.i

lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_vo.dir/src/utils.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/utils.cpp -o CMakeFiles/collision_vo.dir/src/utils.cpp.s

lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o: lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make
lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o: ../../lib/collision_vo/src/admissible_velocities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/admissible_velocities.cpp

lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/admissible_velocities.cpp > CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.i

lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/admissible_velocities.cpp -o CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.s

lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o: lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make
lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o: ../../lib/collision_vo/src/convex_hull.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/convex_hull.cpp

lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_vo.dir/src/convex_hull.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/convex_hull.cpp > CMakeFiles/collision_vo.dir/src/convex_hull.cpp.i

lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_vo.dir/src/convex_hull.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/convex_hull.cpp -o CMakeFiles/collision_vo.dir/src/convex_hull.cpp.s

lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.o: lib/collision_vo/CMakeFiles/collision_vo.dir/flags.make
lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.o: ../../lib/collision_vo/src/buffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.o"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_vo.dir/src/buffer.cpp.o -c /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/buffer.cpp

lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_vo.dir/src/buffer.cpp.i"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/buffer.cpp > CMakeFiles/collision_vo.dir/src/buffer.cpp.i

lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_vo.dir/src/buffer.cpp.s"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && /bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo/src/buffer.cpp -o CMakeFiles/collision_vo.dir/src/buffer.cpp.s

# Object files for target collision_vo
collision_vo_OBJECTS = \
"CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o" \
"CMakeFiles/collision_vo.dir/src/utils.cpp.o" \
"CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o" \
"CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o" \
"CMakeFiles/collision_vo.dir/src/buffer.cpp.o"

# External object files for target collision_vo
collision_vo_EXTERNAL_OBJECTS =

lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/src/collision_vo.cpp.o
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/src/utils.cpp.o
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/src/admissible_velocities.cpp.o
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/src/convex_hull.cpp.o
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/src/buffer.cpp.o
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/build.make
lib/collision_vo/libcollision_vo.a: lib/collision_vo/CMakeFiles/collision_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libcollision_vo.a"
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && $(CMAKE_COMMAND) -P CMakeFiles/collision_vo.dir/cmake_clean_target.cmake
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/collision_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/collision_vo/CMakeFiles/collision_vo.dir/build: lib/collision_vo/libcollision_vo.a

.PHONY : lib/collision_vo/CMakeFiles/collision_vo.dir/build

lib/collision_vo/CMakeFiles/collision_vo.dir/clean:
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo && $(CMAKE_COMMAND) -P CMakeFiles/collision_vo.dir/cmake_clean.cmake
.PHONY : lib/collision_vo/CMakeFiles/collision_vo.dir/clean

lib/collision_vo/CMakeFiles/collision_vo.dir/depend:
	cd /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jonathan/Desktop/5G_uav_sim /home/jonathan/Desktop/5G_uav_sim/lib/collision_vo /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo /home/jonathan/Desktop/5G_uav_sim/5G-air-simulator/build/lib/collision_vo/CMakeFiles/collision_vo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/collision_vo/CMakeFiles/collision_vo.dir/depend

