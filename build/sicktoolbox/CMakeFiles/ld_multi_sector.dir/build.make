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
CMAKE_SOURCE_DIR = /home/anushka/ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anushka/ws/build

# Include any dependencies generated for this target.
include sicktoolbox/CMakeFiles/ld_multi_sector.dir/depend.make

# Include the progress variables for this target.
include sicktoolbox/CMakeFiles/ld_multi_sector.dir/progress.make

# Include the compile flags for this target's objects.
include sicktoolbox/CMakeFiles/ld_multi_sector.dir/flags.make

sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o: sicktoolbox/CMakeFiles/ld_multi_sector.dir/flags.make
sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o: /home/anushka/ws/src/sicktoolbox/c++/examples/ld/ld_multi_sector/src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anushka/ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o"
	cd /home/anushka/ws/build/sicktoolbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o -c /home/anushka/ws/src/sicktoolbox/c++/examples/ld/ld_multi_sector/src/main.cc

sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.i"
	cd /home/anushka/ws/build/sicktoolbox && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anushka/ws/src/sicktoolbox/c++/examples/ld/ld_multi_sector/src/main.cc > CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.i

sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.s"
	cd /home/anushka/ws/build/sicktoolbox && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anushka/ws/src/sicktoolbox/c++/examples/ld/ld_multi_sector/src/main.cc -o CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.s

# Object files for target ld_multi_sector
ld_multi_sector_OBJECTS = \
"CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o"

# External object files for target ld_multi_sector
ld_multi_sector_EXTERNAL_OBJECTS =

/home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector: sicktoolbox/CMakeFiles/ld_multi_sector.dir/c++/examples/ld/ld_multi_sector/src/main.cc.o
/home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector: sicktoolbox/CMakeFiles/ld_multi_sector.dir/build.make
/home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector: /home/anushka/ws/devel/lib/libSickLD.so
/home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector: sicktoolbox/CMakeFiles/ld_multi_sector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anushka/ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector"
	cd /home/anushka/ws/build/sicktoolbox && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ld_multi_sector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sicktoolbox/CMakeFiles/ld_multi_sector.dir/build: /home/anushka/ws/devel/lib/sicktoolbox/ld_multi_sector

.PHONY : sicktoolbox/CMakeFiles/ld_multi_sector.dir/build

sicktoolbox/CMakeFiles/ld_multi_sector.dir/clean:
	cd /home/anushka/ws/build/sicktoolbox && $(CMAKE_COMMAND) -P CMakeFiles/ld_multi_sector.dir/cmake_clean.cmake
.PHONY : sicktoolbox/CMakeFiles/ld_multi_sector.dir/clean

sicktoolbox/CMakeFiles/ld_multi_sector.dir/depend:
	cd /home/anushka/ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anushka/ws/src /home/anushka/ws/src/sicktoolbox /home/anushka/ws/build /home/anushka/ws/build/sicktoolbox /home/anushka/ws/build/sicktoolbox/CMakeFiles/ld_multi_sector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sicktoolbox/CMakeFiles/ld_multi_sector.dir/depend

