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
CMAKE_SOURCE_DIR = /home/anton202/r2d2_general_ws/tcp_client_rx_tx

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build

# Include any dependencies generated for this target.
include CMakeFiles/very_simple_tcp_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/very_simple_tcp_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/very_simple_tcp_server.dir/flags.make

CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o: CMakeFiles/very_simple_tcp_server.dir/flags.make
CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o: ../very_simple_tcp_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o -c /home/anton202/r2d2_general_ws/tcp_client_rx_tx/very_simple_tcp_server.cpp

CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anton202/r2d2_general_ws/tcp_client_rx_tx/very_simple_tcp_server.cpp > CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.i

CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anton202/r2d2_general_ws/tcp_client_rx_tx/very_simple_tcp_server.cpp -o CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.s

CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o: CMakeFiles/very_simple_tcp_server.dir/flags.make
CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o: ../umba_crc_table.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o"
	/usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o   -c /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c

CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.i"
	/usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c > CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.i

CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.s"
	/usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c -o CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.s

# Object files for target very_simple_tcp_server
very_simple_tcp_server_OBJECTS = \
"CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o" \
"CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o"

# External object files for target very_simple_tcp_server
very_simple_tcp_server_EXTERNAL_OBJECTS =

very_simple_tcp_server: CMakeFiles/very_simple_tcp_server.dir/very_simple_tcp_server.cpp.o
very_simple_tcp_server: CMakeFiles/very_simple_tcp_server.dir/umba_crc_table.c.o
very_simple_tcp_server: CMakeFiles/very_simple_tcp_server.dir/build.make
very_simple_tcp_server: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
very_simple_tcp_server: CMakeFiles/very_simple_tcp_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable very_simple_tcp_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/very_simple_tcp_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/very_simple_tcp_server.dir/build: very_simple_tcp_server

.PHONY : CMakeFiles/very_simple_tcp_server.dir/build

CMakeFiles/very_simple_tcp_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/very_simple_tcp_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/very_simple_tcp_server.dir/clean

CMakeFiles/very_simple_tcp_server.dir/depend:
	cd /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anton202/r2d2_general_ws/tcp_client_rx_tx /home/anton202/r2d2_general_ws/tcp_client_rx_tx /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles/very_simple_tcp_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/very_simple_tcp_server.dir/depend

