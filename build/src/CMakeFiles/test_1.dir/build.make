# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/jeremy/ParaSpline

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jeremy/ParaSpline/build

# Include any dependencies generated for this target.
include src/CMakeFiles/test_1.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/test_1.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/test_1.dir/flags.make

src/CMakeFiles/test_1.dir/test.cpp.o: src/CMakeFiles/test_1.dir/flags.make
src/CMakeFiles/test_1.dir/test.cpp.o: ../src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jeremy/ParaSpline/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/test_1.dir/test.cpp.o"
	cd /home/jeremy/ParaSpline/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_1.dir/test.cpp.o -c /home/jeremy/ParaSpline/src/test.cpp

src/CMakeFiles/test_1.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_1.dir/test.cpp.i"
	cd /home/jeremy/ParaSpline/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jeremy/ParaSpline/src/test.cpp > CMakeFiles/test_1.dir/test.cpp.i

src/CMakeFiles/test_1.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_1.dir/test.cpp.s"
	cd /home/jeremy/ParaSpline/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jeremy/ParaSpline/src/test.cpp -o CMakeFiles/test_1.dir/test.cpp.s

src/CMakeFiles/test_1.dir/test.cpp.o.requires:

.PHONY : src/CMakeFiles/test_1.dir/test.cpp.o.requires

src/CMakeFiles/test_1.dir/test.cpp.o.provides: src/CMakeFiles/test_1.dir/test.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_1.dir/build.make src/CMakeFiles/test_1.dir/test.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_1.dir/test.cpp.o.provides

src/CMakeFiles/test_1.dir/test.cpp.o.provides.build: src/CMakeFiles/test_1.dir/test.cpp.o


# Object files for target test_1
test_1_OBJECTS = \
"CMakeFiles/test_1.dir/test.cpp.o"

# External object files for target test_1
test_1_EXTERNAL_OBJECTS =

bin/test_1: src/CMakeFiles/test_1.dir/test.cpp.o
bin/test_1: src/CMakeFiles/test_1.dir/build.make
bin/test_1: lib/libheader.a
bin/test_1: src/CMakeFiles/test_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jeremy/ParaSpline/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/test_1"
	cd /home/jeremy/ParaSpline/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/test_1.dir/build: bin/test_1

.PHONY : src/CMakeFiles/test_1.dir/build

src/CMakeFiles/test_1.dir/requires: src/CMakeFiles/test_1.dir/test.cpp.o.requires

.PHONY : src/CMakeFiles/test_1.dir/requires

src/CMakeFiles/test_1.dir/clean:
	cd /home/jeremy/ParaSpline/build/src && $(CMAKE_COMMAND) -P CMakeFiles/test_1.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/test_1.dir/clean

src/CMakeFiles/test_1.dir/depend:
	cd /home/jeremy/ParaSpline/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jeremy/ParaSpline /home/jeremy/ParaSpline/src /home/jeremy/ParaSpline/build /home/jeremy/ParaSpline/build/src /home/jeremy/ParaSpline/build/src/CMakeFiles/test_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/test_1.dir/depend

