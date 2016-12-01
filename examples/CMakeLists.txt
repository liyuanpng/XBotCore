# 
#  Copyright (C) 2016 IIT-ADVR
#  Author: Luca Muratore
#  email: luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

#minimum cmake required version
cmake_minimum_required(VERSION 3.0)
# generic_tutorial project
project(XCM_examples)

###################
## Configuration ##
###################

#enable C++ 11 : try with two different flags
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find the required packages

#############
## Include ##
#############
include_directories(include)

###########
## Build ##
###########
add_library(IkExample src/ik_example.cpp)
add_library(HomingExample src/homing_example.cpp)
add_library(JointTrajectoryGenerator src/joint_space_demo_trajectory.cpp)
add_library(PlayJointSpaceDemo src/play_joint_space_demo.cpp)
add_library(TwoPriorityRtIk src/twopriority_rt_ik.cpp)
add_library(TwoPriorityRtIkPlugin src/twopriorirty_ik_plugin.cpp)

##########
## Link ##
##########
target_link_libraries(IkExample XCM)
target_link_libraries(HomingExample XCM)
target_link_libraries(JointTrajectoryGenerator XCM)
target_link_libraries(PlayJointSpaceDemo XCM JointTrajectoryGenerator)
target_link_libraries(TwoPriorityRtIk XCM)
target_link_libraries(TwoPriorityRtIkPlugin XCM TwoPriorityRtIk JointTrajectoryGenerator)

#############
## Install ##
#############


set(VARS_PREFIX "IkExample")

set(IkExample_MAJOR_VERSION 0)
set(IkExample_MINOR_VERSION 1)
set(IkExample_PATCH_VERSION 0)
set(IkExample_VERSION ${IkExample_MAJOR_VERSION}.${IkExample_MINOR_VERSION}.${IkExample_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(IkExample PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(IkExample PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS IkExample
        EXPORT IkExample
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all IkExample dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS IkExample)


include(InstallBasicPackageFiles)
install_basic_package_files(IkExample VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
                                    
set(VARS_PREFIX "HomingExample")

set(HomingExample_MAJOR_VERSION 0)
set(HomingExample_MINOR_VERSION 1)
set(HomingExample_PATCH_VERSION 0)
set(HomingExample_VERSION ${HomingExample_MAJOR_VERSION}.${HomingExample_MINOR_VERSION}.${HomingExample_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(HomingExample PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(HomingExample PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS HomingExample
        EXPORT HomingExample
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all HomingExample dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS HomingExample)


include(InstallBasicPackageFiles)
install_basic_package_files(HomingExample VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
set(VARS_PREFIX "JointTrajectoryGenerator")

set(JointTrajectoryGenerator_MAJOR_VERSION 0)
set(JointTrajectoryGenerator_MINOR_VERSION 1)
set(JointTrajectoryGenerator_PATCH_VERSION 0)
set(JointTrajectoryGenerator_VERSION ${JointTrajectoryGenerator_MAJOR_VERSION}.${JointTrajectoryGenerator_MINOR_VERSION}.${JointTrajectoryGenerator_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(JointTrajectoryGenerator PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(JointTrajectoryGenerator PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS JointTrajectoryGenerator
        EXPORT JointTrajectoryGenerator
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all JointTrajectoryGenerator dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS JointTrajectoryGenerator)


include(InstallBasicPackageFiles)
install_basic_package_files(JointTrajectoryGenerator VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
set(VARS_PREFIX "PlayJointSpaceDemo")

set(PlayJointSpaceDemo_MAJOR_VERSION 0)
set(PlayJointSpaceDemo_MINOR_VERSION 1)
set(PlayJointSpaceDemo_PATCH_VERSION 0)
set(PlayJointSpaceDemo_VERSION ${PlayJointSpaceDemo_MAJOR_VERSION}.${PlayJointSpaceDemo_MINOR_VERSION}.${PlayJointSpaceDemo_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(PlayJointSpaceDemo PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(PlayJointSpaceDemo PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS PlayJointSpaceDemo
        EXPORT PlayJointSpaceDemo
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all PlayJointSpaceDemo dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS PlayJointSpaceDemo)


include(InstallBasicPackageFiles)
install_basic_package_files(PlayJointSpaceDemo VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
set(VARS_PREFIX "TwoPriorityRtIk")

set(TwoPriorityRtIk_MAJOR_VERSION 0)
set(TwoPriorityRtIk_MINOR_VERSION 1)
set(TwoPriorityRtIk_PATCH_VERSION 0)
set(TwoPriorityRtIk_VERSION ${TwoPriorityRtIk_MAJOR_VERSION}.${TwoPriorityRtIk_MINOR_VERSION}.${TwoPriorityRtIk_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(TwoPriorityRtIk PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(TwoPriorityRtIk PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS TwoPriorityRtIk
        EXPORT TwoPriorityRtIk
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all TwoPriorityRtIk dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS TwoPriorityRtIk)


include(InstallBasicPackageFiles)
install_basic_package_files(TwoPriorityRtIk VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

set(VARS_PREFIX "TwoPriorityRtIkPlugin")

set(TwoPriorityRtIkPlugin_MAJOR_VERSION 0)
set(TwoPriorityRtIkPlugin_MINOR_VERSION 1)
set(TwoPriorityRtIkPlugin_PATCH_VERSION 0)
set(TwoPriorityRtIkPlugin_VERSION ${TwoPriorityRtIkPlugin_MAJOR_VERSION}.${TwoPriorityRtIkPlugin_MINOR_VERSION}.${TwoPriorityRtIkPlugin_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(TwoPriorityRtIkPlugin PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(TwoPriorityRtIkPlugin PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS TwoPriorityRtIkPlugin
        EXPORT TwoPriorityRtIkPlugin
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all TwoPriorityRtIkPlugin dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS TwoPriorityRtIkPlugin)


include(InstallBasicPackageFiles)
install_basic_package_files(TwoPriorityRtIkPlugin VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
                                    
include(AddUninstallTarget)
