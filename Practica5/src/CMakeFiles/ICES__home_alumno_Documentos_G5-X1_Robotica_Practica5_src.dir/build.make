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
CMAKE_COMMAND = /snap/clion/178/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/178/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alumno/Documentos/G5-X1/Robotica/Practica5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alumno/Documentos/G5-X1/Robotica/Practica5

# Utility rule file for ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/progress.make

ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src: src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /opt/robocomp//interfaces/IDSLs/CommonBehavior.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/CommonBehavior.ice"
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && robocompdsl /opt/robocomp//interfaces/IDSLs/CommonBehavior.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /opt/robocomp//interfaces/IDSLs/DifferentialRobot.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/DifferentialRobot.ice"
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && robocompdsl /opt/robocomp//interfaces/IDSLs/DifferentialRobot.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/DifferentialRobot.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /opt/robocomp//interfaces/IDSLs/FullPoseEstimation.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/FullPoseEstimation.ice"
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && robocompdsl /opt/robocomp//interfaces/IDSLs/FullPoseEstimation.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/FullPoseEstimation.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /opt/robocomp//interfaces/IDSLs/GenericBase.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/GenericBase.ice"
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && robocompdsl /opt/robocomp//interfaces/IDSLs/GenericBase.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /opt/robocomp//interfaces/IDSLs/Laser.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/Laser.ice"
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && robocompdsl /opt/robocomp//interfaces/IDSLs/Laser.idsl /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/Laser.ice
.PHONY : ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/build: ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src
.PHONY : src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/build

src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/clean:
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/clean

src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/depend:
	cd /home/alumno/Documentos/G5-X1/Robotica/Practica5 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alumno/Documentos/G5-X1/Robotica/Practica5 /home/alumno/Documentos/G5-X1/Robotica/Practica5/src /home/alumno/Documentos/G5-X1/Robotica/Practica5 /home/alumno/Documentos/G5-X1/Robotica/Practica5/src /home/alumno/Documentos/G5-X1/Robotica/Practica5/src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ICES__home_alumno_Documentos_G5-X1_Robotica_Practica5_src.dir/depend

