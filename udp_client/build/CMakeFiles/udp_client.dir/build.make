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
CMAKE_SOURCE_DIR = /home/anton202/r2d2_general_ws/udp_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anton202/r2d2_general_ws/udp_client/build

# Include any dependencies generated for this target.
include CMakeFiles/udp_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/udp_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/udp_client.dir/flags.make

CMakeFiles/udp_client.dir/udp_client.cpp.o: CMakeFiles/udp_client.dir/flags.make
CMakeFiles/udp_client.dir/udp_client.cpp.o: ../udp_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/udp_client/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/udp_client.dir/udp_client.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/udp_client.dir/udp_client.cpp.o -c /home/anton202/r2d2_general_ws/udp_client/udp_client.cpp

CMakeFiles/udp_client.dir/udp_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/udp_client.dir/udp_client.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anton202/r2d2_general_ws/udp_client/udp_client.cpp > CMakeFiles/udp_client.dir/udp_client.cpp.i

CMakeFiles/udp_client.dir/udp_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/udp_client.dir/udp_client.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anton202/r2d2_general_ws/udp_client/udp_client.cpp -o CMakeFiles/udp_client.dir/udp_client.cpp.s

CMakeFiles/udp_client.dir/umba_crc_table.c.o: CMakeFiles/udp_client.dir/flags.make
CMakeFiles/udp_client.dir/umba_crc_table.c.o: ../umba_crc_table.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/udp_client/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/udp_client.dir/umba_crc_table.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/udp_client.dir/umba_crc_table.c.o   -c /home/anton202/r2d2_general_ws/udp_client/umba_crc_table.c

CMakeFiles/udp_client.dir/umba_crc_table.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/udp_client.dir/umba_crc_table.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/anton202/r2d2_general_ws/udp_client/umba_crc_table.c > CMakeFiles/udp_client.dir/umba_crc_table.c.i

CMakeFiles/udp_client.dir/umba_crc_table.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/udp_client.dir/umba_crc_table.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/anton202/r2d2_general_ws/udp_client/umba_crc_table.c -o CMakeFiles/udp_client.dir/umba_crc_table.c.s

# Object files for target udp_client
udp_client_OBJECTS = \
"CMakeFiles/udp_client.dir/udp_client.cpp.o" \
"CMakeFiles/udp_client.dir/umba_crc_table.c.o"

# External object files for target udp_client
udp_client_EXTERNAL_OBJECTS =

udp_client: CMakeFiles/udp_client.dir/udp_client.cpp.o
udp_client: CMakeFiles/udp_client.dir/umba_crc_table.c.o
udp_client: CMakeFiles/udp_client.dir/build.make
udp_client: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
udp_client: CMakeFiles/udp_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anton202/r2d2_general_ws/udp_client/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable udp_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/udp_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/udp_client.dir/build: udp_client

.PHONY : CMakeFiles/udp_client.dir/build

CMakeFiles/udp_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/udp_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/udp_client.dir/clean

CMakeFiles/udp_client.dir/depend:
	cd /home/anton202/r2d2_general_ws/udp_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anton202/r2d2_general_ws/udp_client /home/anton202/r2d2_general_ws/udp_client /home/anton202/r2d2_general_ws/udp_client/build /home/anton202/r2d2_general_ws/udp_client/build /home/anton202/r2d2_general_ws/udp_client/build/CMakeFiles/udp_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/udp_client.dir/depend

