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

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)


include(ExternalProject)
project(XCM)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

# C++ 11
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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)

# find first YCM in order to have ${YCM_MODULE_PATH} defined for 3rd party software like Eigen3
find_package(YCM REQUIRED)

find_package(XBotInterface REQUIRED)

find_package(XBotLogger REQUIRED)
find_package(XBotCore-interfaces REQUIRED)


option(BUILD_TESTS "Compile XCM tests" FALSE)



include_directories(include ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            )


# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XCM_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

add_library(XCM SHARED ${XBotInterface_INCLUDES}
                        src/XBotControlPlugin.cpp)

target_link_libraries(XCM PUBLIC ${XBotInterface_LIBRARIES}
                                 ${XBotLogger_LIBRARIES}
                                 ${XBotCore-interfaces_LIBRARIES}
                                 )


target_include_directories(XCM PUBLIC
                            ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            )


########################################################################

set(VARS_PREFIX "XCM")

set(XCM_MAJOR_VERSION 0)
set(XCM_MINOR_VERSION 1)
set(XCM_PATCH_VERSION 0)
set(XCM_VERSION ${XCM_MAJOR_VERSION}.${XCM_MINOR_VERSION}.${XCM_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XCM PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XCM PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS XCM
        EXPORT XCM
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XCM dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XCM)


include(InstallBasicPackageFiles)
install_basic_package_files(XCM VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

# doc
optional_build(doc doc ON)
# examples
optional_build(examples examples ON)

#######################
# Add Testing target  #
#######################
if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()
