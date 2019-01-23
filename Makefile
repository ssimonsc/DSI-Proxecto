# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/pi/Navio2/C++/Examples/DDS3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Navio2/C++/Examples/DDS3

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: install/local

.PHONY : install/local/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: install/strip

.PHONY : install/strip/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Navio2/C++/Examples/DDS3/CMakeFiles /home/pi/Navio2/C++/Examples/DDS3/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Navio2/C++/Examples/DDS3/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named pid

# Build rule for target.
pid: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 pid
.PHONY : pid

# fast build rule for target.
pid/fast:
	$(MAKE) -f CMakeFiles/pid.dir/build.make CMakeFiles/pid.dir/build
.PHONY : pid/fast

#=============================================================================
# Target rules for targets named HelloWorldExample

# Build rule for target.
HelloWorldExample: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 HelloWorldExample
.PHONY : HelloWorldExample

# fast build rule for target.
HelloWorldExample/fast:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/build
.PHONY : HelloWorldExample/fast

#=============================================================================
# Target rules for targets named navio

# Build rule for target.
navio: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 navio
.PHONY : navio

# fast build rule for target.
navio/fast:
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/build
.PHONY : navio/fast

#=============================================================================
# Target rules for targets named common

# Build rule for target.
common: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 common
.PHONY : common

# fast build rule for target.
common/fast:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/build
.PHONY : common/fast

HelloWorld.o: HelloWorld.cxx.o

.PHONY : HelloWorld.o

# target to build an object file
HelloWorld.cxx.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld.cxx.o
.PHONY : HelloWorld.cxx.o

HelloWorld.i: HelloWorld.cxx.i

.PHONY : HelloWorld.i

# target to preprocess a source file
HelloWorld.cxx.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld.cxx.i
.PHONY : HelloWorld.cxx.i

HelloWorld.s: HelloWorld.cxx.s

.PHONY : HelloWorld.s

# target to generate assembly for a file
HelloWorld.cxx.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld.cxx.s
.PHONY : HelloWorld.cxx.s

HelloWorldPubSubTypes.o: HelloWorldPubSubTypes.cxx.o

.PHONY : HelloWorldPubSubTypes.o

# target to build an object file
HelloWorldPubSubTypes.cxx.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPubSubTypes.cxx.o
.PHONY : HelloWorldPubSubTypes.cxx.o

HelloWorldPubSubTypes.i: HelloWorldPubSubTypes.cxx.i

.PHONY : HelloWorldPubSubTypes.i

# target to preprocess a source file
HelloWorldPubSubTypes.cxx.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPubSubTypes.cxx.i
.PHONY : HelloWorldPubSubTypes.cxx.i

HelloWorldPubSubTypes.s: HelloWorldPubSubTypes.cxx.s

.PHONY : HelloWorldPubSubTypes.s

# target to generate assembly for a file
HelloWorldPubSubTypes.cxx.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPubSubTypes.cxx.s
.PHONY : HelloWorldPubSubTypes.cxx.s

HelloWorldPublisher.o: HelloWorldPublisher.cpp.o

.PHONY : HelloWorldPublisher.o

# target to build an object file
HelloWorldPublisher.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPublisher.cpp.o
.PHONY : HelloWorldPublisher.cpp.o

HelloWorldPublisher.i: HelloWorldPublisher.cpp.i

.PHONY : HelloWorldPublisher.i

# target to preprocess a source file
HelloWorldPublisher.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPublisher.cpp.i
.PHONY : HelloWorldPublisher.cpp.i

HelloWorldPublisher.s: HelloWorldPublisher.cpp.s

.PHONY : HelloWorldPublisher.s

# target to generate assembly for a file
HelloWorldPublisher.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldPublisher.cpp.s
.PHONY : HelloWorldPublisher.cpp.s

HelloWorldSubscriber.o: HelloWorldSubscriber.cpp.o

.PHONY : HelloWorldSubscriber.o

# target to build an object file
HelloWorldSubscriber.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldSubscriber.cpp.o
.PHONY : HelloWorldSubscriber.cpp.o

HelloWorldSubscriber.i: HelloWorldSubscriber.cpp.i

.PHONY : HelloWorldSubscriber.i

# target to preprocess a source file
HelloWorldSubscriber.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldSubscriber.cpp.i
.PHONY : HelloWorldSubscriber.cpp.i

HelloWorldSubscriber.s: HelloWorldSubscriber.cpp.s

.PHONY : HelloWorldSubscriber.s

# target to generate assembly for a file
HelloWorldSubscriber.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorldSubscriber.cpp.s
.PHONY : HelloWorldSubscriber.cpp.s

HelloWorld_main.o: HelloWorld_main.cpp.o

.PHONY : HelloWorld_main.o

# target to build an object file
HelloWorld_main.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld_main.cpp.o
.PHONY : HelloWorld_main.cpp.o

HelloWorld_main.i: HelloWorld_main.cpp.i

.PHONY : HelloWorld_main.i

# target to preprocess a source file
HelloWorld_main.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld_main.cpp.i
.PHONY : HelloWorld_main.cpp.i

HelloWorld_main.s: HelloWorld_main.cpp.s

.PHONY : HelloWorld_main.s

# target to generate assembly for a file
HelloWorld_main.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/HelloWorld_main.cpp.s
.PHONY : HelloWorld_main.cpp.s

detectorConos.o: detectorConos.cpp.o

.PHONY : detectorConos.o

# target to build an object file
detectorConos.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/detectorConos.cpp.o
.PHONY : detectorConos.cpp.o

detectorConos.i: detectorConos.cpp.i

.PHONY : detectorConos.i

# target to preprocess a source file
detectorConos.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/detectorConos.cpp.i
.PHONY : detectorConos.cpp.i

detectorConos.s: detectorConos.cpp.s

.PHONY : detectorConos.s

# target to generate assembly for a file
detectorConos.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/detectorConos.cpp.s
.PHONY : detectorConos.cpp.s

home/pi/Navio2/C++/Navio/Common/I2Cdev.o: home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.o

home/pi/Navio2/C++/Navio/Common/I2Cdev.i: home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.i

home/pi/Navio2/C++/Navio/Common/I2Cdev.s: home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/I2Cdev.cpp.s

home/pi/Navio2/C++/Navio/Common/MPU9250.o: home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.o

home/pi/Navio2/C++/Navio/Common/MPU9250.i: home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.i

home/pi/Navio2/C++/Navio/Common/MPU9250.s: home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/MPU9250.cpp.s

home/pi/Navio2/C++/Navio/Common/MS5611.o: home/pi/Navio2/C++/Navio/Common/MS5611.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/MS5611.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MS5611.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.cpp.o

home/pi/Navio2/C++/Navio/Common/MS5611.i: home/pi/Navio2/C++/Navio/Common/MS5611.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/MS5611.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MS5611.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.cpp.i

home/pi/Navio2/C++/Navio/Common/MS5611.s: home/pi/Navio2/C++/Navio/Common/MS5611.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/MS5611.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/MS5611.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/MS5611.cpp.s

home/pi/Navio2/C++/Navio/Common/Ublox.o: home/pi/Navio2/C++/Navio/Common/Ublox.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/Ublox.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Ublox.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.cpp.o

home/pi/Navio2/C++/Navio/Common/Ublox.i: home/pi/Navio2/C++/Navio/Common/Ublox.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/Ublox.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Ublox.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.cpp.i

home/pi/Navio2/C++/Navio/Common/Ublox.s: home/pi/Navio2/C++/Navio/Common/Ublox.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/Ublox.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Ublox.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/Ublox.cpp.s

home/pi/Navio2/C++/Navio/Common/Util.o: home/pi/Navio2/C++/Navio/Common/Util.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/Util.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/Util.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Util.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/Util.cpp.o

home/pi/Navio2/C++/Navio/Common/Util.i: home/pi/Navio2/C++/Navio/Common/Util.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/Util.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/Util.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Util.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/Util.cpp.i

home/pi/Navio2/C++/Navio/Common/Util.s: home/pi/Navio2/C++/Navio/Common/Util.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/Util.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/Util.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/Util.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/Util.cpp.s

home/pi/Navio2/C++/Navio/Common/gpio.o: home/pi/Navio2/C++/Navio/Common/gpio.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.o

# target to build an object file
home/pi/Navio2/C++/Navio/Common/gpio.cpp.o:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/gpio.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.cpp.o

home/pi/Navio2/C++/Navio/Common/gpio.i: home/pi/Navio2/C++/Navio/Common/gpio.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Common/gpio.cpp.i:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/gpio.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.cpp.i

home/pi/Navio2/C++/Navio/Common/gpio.s: home/pi/Navio2/C++/Navio/Common/gpio.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Common/gpio.cpp.s:
	$(MAKE) -f CMakeFiles/common.dir/build.make CMakeFiles/common.dir/home/pi/Navio2/C++/Navio/Common/gpio.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Common/gpio.cpp.s

home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.o: home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.o

home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.i: home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.i

home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.s: home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.cpp.s

home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.o: home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.o

home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.i: home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.i

home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.s: home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.cpp.s

home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.o: home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.o

home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.i: home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.i

home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.s: home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.cpp.s

home/pi/Navio2/C++/Navio/Navio2/PWM.o: home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.o

home/pi/Navio2/C++/Navio/Navio2/PWM.i: home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.i

home/pi/Navio2/C++/Navio/Navio2/PWM.s: home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/PWM.cpp.s

home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.o: home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.o

home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.i: home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.i

home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.s: home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.cpp.s

home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.o: home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.o

home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.i: home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.i

home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.s: home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.cpp.s

home/pi/Navio2/C++/Navio/Navio2/RGBled.o: home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.o

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.o

# target to build an object file
home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.o:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.o
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.o
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.o

home/pi/Navio2/C++/Navio/Navio2/RGBled.i: home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.i

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.i

# target to preprocess a source file
home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.i:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.i
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.i
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.i

home/pi/Navio2/C++/Navio/Navio2/RGBled.s: home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.s

.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.s

# target to generate assembly for a file
home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.s:
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.s
	$(MAKE) -f CMakeFiles/navio.dir/build.make CMakeFiles/navio.dir/home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.s
.PHONY : home/pi/Navio2/C++/Navio/Navio2/RGBled.cpp.s

pid.o: pid.cpp.o

.PHONY : pid.o

# target to build an object file
pid.cpp.o:
	$(MAKE) -f CMakeFiles/pid.dir/build.make CMakeFiles/pid.dir/pid.cpp.o
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/pid.cpp.o
.PHONY : pid.cpp.o

pid.i: pid.cpp.i

.PHONY : pid.i

# target to preprocess a source file
pid.cpp.i:
	$(MAKE) -f CMakeFiles/pid.dir/build.make CMakeFiles/pid.dir/pid.cpp.i
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/pid.cpp.i
.PHONY : pid.cpp.i

pid.s: pid.cpp.s

.PHONY : pid.s

# target to generate assembly for a file
pid.cpp.s:
	$(MAKE) -f CMakeFiles/pid.dir/build.make CMakeFiles/pid.dir/pid.cpp.s
	$(MAKE) -f CMakeFiles/HelloWorldExample.dir/build.make CMakeFiles/HelloWorldExample.dir/pid.cpp.s
.PHONY : pid.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... install/local"
	@echo "... install"
	@echo "... rebuild_cache"
	@echo "... install/strip"
	@echo "... edit_cache"
	@echo "... pid"
	@echo "... HelloWorldExample"
	@echo "... list_install_components"
	@echo "... navio"
	@echo "... common"
	@echo "... HelloWorld.o"
	@echo "... HelloWorld.i"
	@echo "... HelloWorld.s"
	@echo "... HelloWorldPubSubTypes.o"
	@echo "... HelloWorldPubSubTypes.i"
	@echo "... HelloWorldPubSubTypes.s"
	@echo "... HelloWorldPublisher.o"
	@echo "... HelloWorldPublisher.i"
	@echo "... HelloWorldPublisher.s"
	@echo "... HelloWorldSubscriber.o"
	@echo "... HelloWorldSubscriber.i"
	@echo "... HelloWorldSubscriber.s"
	@echo "... HelloWorld_main.o"
	@echo "... HelloWorld_main.i"
	@echo "... HelloWorld_main.s"
	@echo "... detectorConos.o"
	@echo "... detectorConos.i"
	@echo "... detectorConos.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/I2Cdev.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/I2Cdev.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/I2Cdev.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/MPU9250.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/MPU9250.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/MPU9250.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/MS5611.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/MS5611.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/MS5611.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/Ublox.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/Ublox.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/Ublox.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/Util.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/Util.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/Util.s"
	@echo "... home/pi/Navio2/C++/Navio/Common/gpio.o"
	@echo "... home/pi/Navio2/C++/Navio/Common/gpio.i"
	@echo "... home/pi/Navio2/C++/Navio/Common/gpio.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/ADC_Navio2.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/LSM9DS1.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/Led_Navio2.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/PWM.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/PWM.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/PWM.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCInput_Navio2.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RCOutput_Navio2.s"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RGBled.o"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RGBled.i"
	@echo "... home/pi/Navio2/C++/Navio/Navio2/RGBled.s"
	@echo "... pid.o"
	@echo "... pid.i"
	@echo "... pid.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

