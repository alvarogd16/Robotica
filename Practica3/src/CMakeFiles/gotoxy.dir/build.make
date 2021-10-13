# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /snap/clion/164/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/164/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alumno/Robotica/Robotica/Practica3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alumno/Robotica/Robotica/Practica3

# Include any dependencies generated for this target.
include src/CMakeFiles/gotoxy.dir/depend.make
# Include the progress variables for this target.
include src/CMakeFiles/gotoxy.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/gotoxy.dir/flags.make

src/CommonBehavior.cpp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "BU ice=>h/cpp: Generating CommonBehavior.h and CommonBehavior.cpp from /home/alumno/Robotica/Robotica/Practica3/src/CommonBehavior.ice"
	cd /home/alumno/Robotica/Robotica/Practica3/src && slice2cpp --underscore /home/alumno/Robotica/Robotica/Practica3/src/CommonBehavior.ice -I/home/alumno/Robotica/Robotica/Practica3/src/ --output-dir .

src/CommonBehavior.h: src/CommonBehavior.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/CommonBehavior.h

src/DifferentialRobot.cpp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "BU ice=>h/cpp: Generating DifferentialRobot.h and DifferentialRobot.cpp from /home/alumno/Robotica/Robotica/Practica3/src/DifferentialRobot.ice"
	cd /home/alumno/Robotica/Robotica/Practica3/src && slice2cpp --underscore /home/alumno/Robotica/Robotica/Practica3/src/DifferentialRobot.ice -I/home/alumno/Robotica/Robotica/Practica3/src/ --output-dir .

src/DifferentialRobot.h: src/DifferentialRobot.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/DifferentialRobot.h

src/GenericBase.cpp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "BU ice=>h/cpp: Generating GenericBase.h and GenericBase.cpp from /home/alumno/Robotica/Robotica/Practica3/src/GenericBase.ice"
	cd /home/alumno/Robotica/Robotica/Practica3/src && slice2cpp --underscore /home/alumno/Robotica/Robotica/Practica3/src/GenericBase.ice -I/home/alumno/Robotica/Robotica/Practica3/src/ --output-dir .

src/GenericBase.h: src/GenericBase.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/GenericBase.h

src/Laser.cpp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "BU ice=>h/cpp: Generating Laser.h and Laser.cpp from /home/alumno/Robotica/Robotica/Practica3/src/Laser.ice"
	cd /home/alumno/Robotica/Robotica/Practica3/src && slice2cpp --underscore /home/alumno/Robotica/Robotica/Practica3/src/Laser.ice -I/home/alumno/Robotica/Robotica/Practica3/src/ --output-dir .

src/Laser.h: src/Laser.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/Laser.h

src/ui_mainUI.h: src/mainUI.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating ui_mainUI.h"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/lib/qt5/bin/uic -o /home/alumno/Robotica/Robotica/Practica3/src/ui_mainUI.h /home/alumno/Robotica/Robotica/Practica3/src/mainUI.ui

src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o: src/gotoxy_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/gotoxy_autogen/mocs_compilation.cpp

src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/gotoxy_autogen/mocs_compilation.cpp > CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.i

src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/gotoxy_autogen/mocs_compilation.cpp -o CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.s

src/CMakeFiles/gotoxy.dir/specificworker.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/specificworker.cpp.o: src/specificworker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/gotoxy.dir/specificworker.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/specificworker.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/specificworker.cpp

src/CMakeFiles/gotoxy.dir/specificworker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/specificworker.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/specificworker.cpp > CMakeFiles/gotoxy.dir/specificworker.cpp.i

src/CMakeFiles/gotoxy.dir/specificworker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/specificworker.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/specificworker.cpp -o CMakeFiles/gotoxy.dir/specificworker.cpp.s

src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.o: src/specificmonitor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/specificmonitor.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/specificmonitor.cpp

src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/specificmonitor.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/specificmonitor.cpp > CMakeFiles/gotoxy.dir/specificmonitor.cpp.i

src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/specificmonitor.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/specificmonitor.cpp -o CMakeFiles/gotoxy.dir/specificmonitor.cpp.s

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o: /opt/robocomp/classes/rapplication/rapplication.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o -c /opt/robocomp/classes/rapplication/rapplication.cpp

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/robocomp/classes/rapplication/rapplication.cpp > CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.i

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/robocomp/classes/rapplication/rapplication.cpp -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.s

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o: /opt/robocomp/classes/sigwatch/sigwatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o -c /opt/robocomp/classes/sigwatch/sigwatch.cpp

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/robocomp/classes/sigwatch/sigwatch.cpp > CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.i

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/robocomp/classes/sigwatch/sigwatch.cpp -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.s

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o: /opt/robocomp/classes/qlog/qlog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o -c /opt/robocomp/classes/qlog/qlog.cpp

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/robocomp/classes/qlog/qlog.cpp > CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.i

src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/robocomp/classes/qlog/qlog.cpp -o CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.s

src/CMakeFiles/gotoxy.dir/main.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/CMakeFiles/gotoxy.dir/main.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/main.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/main.cpp

src/CMakeFiles/gotoxy.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/main.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/main.cpp > CMakeFiles/gotoxy.dir/main.cpp.i

src/CMakeFiles/gotoxy.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/main.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/main.cpp -o CMakeFiles/gotoxy.dir/main.cpp.s

src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.o: src/genericmonitor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/genericmonitor.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/genericmonitor.cpp

src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/genericmonitor.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/genericmonitor.cpp > CMakeFiles/gotoxy.dir/genericmonitor.cpp.i

src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/genericmonitor.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/genericmonitor.cpp -o CMakeFiles/gotoxy.dir/genericmonitor.cpp.s

src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o: src/commonbehaviorI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/commonbehaviorI.cpp

src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/commonbehaviorI.cpp > CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.i

src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/commonbehaviorI.cpp -o CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.s

src/CMakeFiles/gotoxy.dir/genericworker.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/genericworker.cpp.o: src/genericworker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object src/CMakeFiles/gotoxy.dir/genericworker.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/genericworker.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/genericworker.cpp

src/CMakeFiles/gotoxy.dir/genericworker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/genericworker.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/genericworker.cpp > CMakeFiles/gotoxy.dir/genericworker.cpp.i

src/CMakeFiles/gotoxy.dir/genericworker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/genericworker.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/genericworker.cpp -o CMakeFiles/gotoxy.dir/genericworker.cpp.s

src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o: src/CommonBehavior.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/CommonBehavior.cpp

src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/CommonBehavior.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/CommonBehavior.cpp > CMakeFiles/gotoxy.dir/CommonBehavior.cpp.i

src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/CommonBehavior.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/CommonBehavior.cpp -o CMakeFiles/gotoxy.dir/CommonBehavior.cpp.s

src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o: src/DifferentialRobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/DifferentialRobot.cpp

src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/DifferentialRobot.cpp > CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.i

src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/DifferentialRobot.cpp -o CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.s

src/CMakeFiles/gotoxy.dir/GenericBase.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/GenericBase.cpp.o: src/GenericBase.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object src/CMakeFiles/gotoxy.dir/GenericBase.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/GenericBase.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/GenericBase.cpp

src/CMakeFiles/gotoxy.dir/GenericBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/GenericBase.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/GenericBase.cpp > CMakeFiles/gotoxy.dir/GenericBase.cpp.i

src/CMakeFiles/gotoxy.dir/GenericBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/GenericBase.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/GenericBase.cpp -o CMakeFiles/gotoxy.dir/GenericBase.cpp.s

src/CMakeFiles/gotoxy.dir/Laser.cpp.o: src/CMakeFiles/gotoxy.dir/flags.make
src/CMakeFiles/gotoxy.dir/Laser.cpp.o: src/Laser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object src/CMakeFiles/gotoxy.dir/Laser.cpp.o"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gotoxy.dir/Laser.cpp.o -c /home/alumno/Robotica/Robotica/Practica3/src/Laser.cpp

src/CMakeFiles/gotoxy.dir/Laser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotoxy.dir/Laser.cpp.i"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alumno/Robotica/Robotica/Practica3/src/Laser.cpp > CMakeFiles/gotoxy.dir/Laser.cpp.i

src/CMakeFiles/gotoxy.dir/Laser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotoxy.dir/Laser.cpp.s"
	cd /home/alumno/Robotica/Robotica/Practica3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alumno/Robotica/Robotica/Practica3/src/Laser.cpp -o CMakeFiles/gotoxy.dir/Laser.cpp.s

# Object files for target gotoxy
gotoxy_OBJECTS = \
"CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/gotoxy.dir/specificworker.cpp.o" \
"CMakeFiles/gotoxy.dir/specificmonitor.cpp.o" \
"CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o" \
"CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o" \
"CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o" \
"CMakeFiles/gotoxy.dir/main.cpp.o" \
"CMakeFiles/gotoxy.dir/genericmonitor.cpp.o" \
"CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o" \
"CMakeFiles/gotoxy.dir/genericworker.cpp.o" \
"CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o" \
"CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o" \
"CMakeFiles/gotoxy.dir/GenericBase.cpp.o" \
"CMakeFiles/gotoxy.dir/Laser.cpp.o"

# External object files for target gotoxy
gotoxy_EXTERNAL_OBJECTS =

bin/gotoxy: src/CMakeFiles/gotoxy.dir/gotoxy_autogen/mocs_compilation.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/specificworker.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/specificmonitor.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/rapplication/rapplication.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/sigwatch/sigwatch.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/opt/robocomp/classes/qlog/qlog.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/main.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/genericmonitor.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/commonbehaviorI.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/genericworker.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/CommonBehavior.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/DifferentialRobot.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/GenericBase.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/Laser.cpp.o
bin/gotoxy: src/CMakeFiles/gotoxy.dir/build.make
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Sql.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Xml.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5XmlPatterns.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libIce++11.so
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libIceStorm++11.so
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libIce.so
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libIceStorm.so
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.12.8
bin/gotoxy: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
bin/gotoxy: src/CMakeFiles/gotoxy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alumno/Robotica/Robotica/Practica3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Linking CXX executable ../bin/gotoxy"
	cd /home/alumno/Robotica/Robotica/Practica3/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gotoxy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/gotoxy.dir/build: bin/gotoxy
.PHONY : src/CMakeFiles/gotoxy.dir/build

src/CMakeFiles/gotoxy.dir/clean:
	cd /home/alumno/Robotica/Robotica/Practica3/src && $(CMAKE_COMMAND) -P CMakeFiles/gotoxy.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/gotoxy.dir/clean

src/CMakeFiles/gotoxy.dir/depend: src/CommonBehavior.cpp
src/CMakeFiles/gotoxy.dir/depend: src/CommonBehavior.h
src/CMakeFiles/gotoxy.dir/depend: src/DifferentialRobot.cpp
src/CMakeFiles/gotoxy.dir/depend: src/DifferentialRobot.h
src/CMakeFiles/gotoxy.dir/depend: src/GenericBase.cpp
src/CMakeFiles/gotoxy.dir/depend: src/GenericBase.h
src/CMakeFiles/gotoxy.dir/depend: src/Laser.cpp
src/CMakeFiles/gotoxy.dir/depend: src/Laser.h
src/CMakeFiles/gotoxy.dir/depend: src/ui_mainUI.h
	cd /home/alumno/Robotica/Robotica/Practica3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alumno/Robotica/Robotica/Practica3 /home/alumno/Robotica/Robotica/Practica3/src /home/alumno/Robotica/Robotica/Practica3 /home/alumno/Robotica/Robotica/Practica3/src /home/alumno/Robotica/Robotica/Practica3/src/CMakeFiles/gotoxy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/gotoxy.dir/depend

