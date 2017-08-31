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

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

if ( Xenomai_FOUND )
    set(RT_NRT_SRC
        src/XBotRT_thread.cpp
        src/XBotNRT_thread.cpp
        src/XBotRT_ipc.cpp
        )
else()
    set(RT_NRT_SRC
        src/XBotNRT_thread.cpp
        )
endif()


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)

# find first YCM in order to have ${YCM_MODULE_PATH} defined for 3rd party software like Eigen3
find_package(YCM REQUIRED)

find_package(XBotInterface REQUIRED)

find_package(catkin QUIET COMPONENTS roscpp sensor_msgs std_srvs message_generation robot_state_publisher)
find_package(YARP QUIET)
find_package(kdl_parser REQUIRED)


option(BUILD_TESTS "Compile XCM tests" FALSE)

if( ${catkin_FOUND} )

    add_message_files(
    DIRECTORY msg
    FILES
    JointStateAdvr.msg
    CommandAdvr.msg
    )

    add_service_files(
    DIRECTORY srv
    FILES
    advr_controller_joint_names.srv
    cmd_service.srv
    status_service.srv
    )

    generate_messages(
    DEPENDENCIES
    std_msgs
    )

    add_library(AdvrCommandMessage SHARED src/MessageInterfaces/AdvrCommandMessage.cpp)
    target_link_libraries(AdvrCommandMessage ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})

    add_library(AdvrJointStateMessage SHARED src/MessageInterfaces/AdvrJointState.cpp)
    target_link_libraries(AdvrJointStateMessage ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})


    add_dependencies(AdvrCommandMessage XCM_generate_messages_cpp)
    add_dependencies(AdvrJointStateMessage XCM_generate_messages_cpp)

endif()

include_directories(include ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            include/Civet/
                            include/Web/
                            )


# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XCM_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

set(COMMUNICATION_INTERFACE_SOURCE_FILES)
set(COMMUNICATION_INTERFACE_LIBRARIES)

option(ENABLE_YARP "Enable YARP Communication Interface" FALSE)

if(${catkin_FOUND})
    add_definitions(-DUSE_ROS_COMMUNICATION_INTERFACE)
    set(COMMUNICATION_INTERFACE_SOURCE_FILES ${COMMUNICATION_INTERFACE_SOURCE_FILES} src/CommunicationInterfaceROS.cpp)
    set(COMMUNICATION_INTERFACE_LIBRARIES ${COMMUNICATION_INTERFACE_LIBRARIES} ${catkin_LIBRARIES})
endif()

if(${YARP_FOUND} AND ${ENABLE_YARP})

    add_definitions(-DUSE_YARP_COMMUNICATION_INTERFACE)

    list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

    # YARP plugin configuration
    include(YarpPlugin)
    include(YarpInstallationHelpers)

    set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "XBotYarp is always built with dynamic plugins")
    yarp_configure_external_installation(XBotYarp WITH_PLUGINS)

    # add the RPATH support
    include(AddInstallRPATHSupport)

    add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"                                        # Libraries
                              BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"                                        # Binaries
                                       "${CMAKE_INSTALL_PREFIX}/${XBOTYARP_DYNAMIC_PLUGINS_INSTALL_DIR}"     # Plugins
                              USE_LINK_PATH)

    # Prepare a plugin library called "XBotYarp".
    YARP_BEGIN_PLUGIN_LIBRARY(XBotYarp)

    add_subdirectory(src/XBotYARP/XBotMotionControl)
    add_subdirectory(src/XBotYARP/XBotFT)
    add_subdirectory(src/XBotYARP/XBotVN100)

    YARP_END_PLUGIN_LIBRARY(XBotYarp)

    set(COMMUNICATION_INTERFACE_SOURCE_FILES ${COMMUNICATION_INTERFACE_SOURCE_FILES} src/XBotYARP/CommunicationInterfaceYARP.cpp)
    set(COMMUNICATION_INTERFACE_LIBRARIES ${COMMUNICATION_INTERFACE_LIBRARIES} ${YARP_LIBRARIES})




endif()

message(${COMMUNICATION_INTERFACE_SOURCE_FILES})

add_definitions(-DUSE_WEBSOCKET)

add_library(XCM SHARED ${XBotInterface_INCLUDES}
                        src/XDomainCommunication.cpp
                        src/XBotPlugin.cpp
                        src/XBotControlPlugin.cpp
                        src/XBotPluginHandler.cpp
                        src/XBotModuleHandler.cpp
                        src/XBotControlModule.cpp
                        src/XBotCommunicationHandler.cpp
                        src/XBotXDDP.cpp
                        src/XBotESCUtils.cpp
                        src/IOPlugin.cpp
                        src/XBotPluginStatus.cpp
                        src/CommunicationInterfaceWebServer.cpp
                        ${COMMUNICATION_INTERFACE_SOURCE_FILES}
                        ${RT_NRT_SRC}
                        src/Civet/CivetServer.cpp
                        src/Civet/civetweb.c
                        src/Web/HttpCivetHandler.cpp
                        src/Web/HttpHandler.cpp
                        src/Web/SharedData.cpp
                        src/Web/WebRobotState.cpp
                        src/Web/WebSocketCivetHandler.cpp
                        )
                        
add_library(XCM_no_xeno SHARED ${XBotInterface_INCLUDES}
                        src/XDomainCommunication.cpp
                        src/XBotPlugin.cpp
                        src/XBotControlPlugin.cpp
                        src/XBotPluginHandler.cpp
                        src/XBotModuleHandler.cpp
                        src/XBotControlModule.cpp
                        src/XBotCommunicationHandler.cpp
                        src/XBotXDDP.cpp
                        src/XBotESCUtils.cpp
                        src/IOPlugin.cpp
                        src/XBotPluginStatus.cpp
                        src/CommunicationInterfaceWebServer.cpp
                        ${COMMUNICATION_INTERFACE_SOURCE_FILES}
                        src/Civet//CivetServer.cpp
                        src/Civet/civetweb.c
                        src/Web/HttpCivetHandler.cpp
                        src/Web/WebSocketCivetHandler.cpp
                        src/Web/HttpHandler.cpp
                        src/Web/SharedData.cpp
                        src/Web/WebRobotState.cpp
                        src/XBotNRT_thread.cpp
                        )                        

if(${catkin_FOUND})
    add_dependencies(XCM AdvrJointStateMessage)
    add_dependencies(XCM AdvrCommandMessage)
    
    add_dependencies(XCM_no_xeno XCM)
endif()


target_link_libraries(XCM PUBLIC ${XBotInterface_LIBRARIES}
                                 ${XBotCore-interfaces_LIBRARIES}
                                 ${COMMUNICATION_INTERFACE_LIBRARIES}
                                 )


target_include_directories( XCM PUBLIC
                            ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            )
                            
target_link_libraries(XCM_no_xeno PUBLIC ${XBotInterface_LIBRARIES}
                                 ${XBotCore-interfaces_LIBRARIES}
                                 ${COMMUNICATION_INTERFACE_LIBRARIES}
                                 )


target_include_directories( XCM_no_xeno PUBLIC
                            ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            )                            

if ( Xenomai_FOUND )
    set_xeno_flags(XCM)
endif()

add_executable(DummyMain src/DummyMain.cpp)
target_link_libraries(DummyMain XCM)

install(TARGETS DummyMain
        EXPORT  DummyMain
        RUNTIME DESTINATION bin)

add_executable(CommunicationHandler src/XBotCommunicationMain.cpp)
target_link_libraries(CommunicationHandler XCM ${COMMUNICATION_INTERFACE_LIBRARIES})

install(TARGETS CommunicationHandler
        EXPORT  CommunicationHandler
        RUNTIME DESTINATION bin)

add_executable(NRTDeployer src/NRTDeployer.cpp src/NRTDeployerMain.cpp)
target_link_libraries(NRTDeployer XCM_no_xeno)

install(TARGETS NRTDeployer
        EXPORT  NRTDeployer
        RUNTIME DESTINATION bin)


if ( Xenomai_FOUND )
    set_xeno_flags(XCM)
endif()


add_subdirectory(src/XBotPlugin)


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
                                    
########################################################################

set(VARS_PREFIX "XCM_no_xeno")

set(XCM_no_xeno_MAJOR_VERSION 0)
set(XCM_no_xeno_MINOR_VERSION 1)
set(XCM_no_xeno_PATCH_VERSION 0)
set(XCM_no_xeno_VERSION ${XCM_no_xeno_MAJOR_VERSION}.${XCM_no_xeno_MINOR_VERSION}.${XCM_no_xeno_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XCM_no_xeno PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XCM_no_xeno PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS XCM_no_xeno
        EXPORT XCM_no_xeno
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XCM_no_xeno dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XCM_no_xeno)


include(InstallBasicPackageFiles)
install_basic_package_files(XCM_no_xeno VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)  
########################################################################                                    

if( ${catkin_FOUND} )
########################################################################

set(VARS_PREFIX "AdvrCommandMessage")

set(AdvrCommandMessage_MAJOR_VERSION 0)
set(AdvrCommandMessage_MINOR_VERSION 1)
set(AdvrCommandMessage_PATCH_VERSION 0)
set(AdvrCommandMessage_VERSION ${AdvrCommandMessage_MAJOR_VERSION}.${AdvrCommandMessage_MINOR_VERSION}.${AdvrCommandMessage_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(AdvrCommandMessage PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(AdvrCommandMessage PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS AdvrCommandMessage
        EXPORT AdvrCommandMessage
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all AdvrCommandMessage dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS AdvrCommandMessage)


include(InstallBasicPackageFiles)
install_basic_package_files(AdvrCommandMessage VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

########################################################################

set(VARS_PREFIX "AdvrJointStateMessage")

set(AdvrJointStateMessage_MAJOR_VERSION 0)
set(AdvrJointStateMessage_MINOR_VERSION 1)
set(AdvrJointStateMessage_PATCH_VERSION 0)
set(AdvrJointStateMessage_VERSION ${AdvrJointStateMessage_MAJOR_VERSION}.${AdvrJointStateMessage_MINOR_VERSION}.${AdvrJointStateMessage_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(AdvrJointStateMessage PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(AdvrJointStateMessage PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS AdvrJointStateMessage
        EXPORT AdvrJointStateMessage
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all AdvrJointStateMessage dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS AdvrJointStateMessage)


include(InstallBasicPackageFiles)
install_basic_package_files(AdvrJointStateMessage VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

endif( ${catkin_FOUND} )


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

# install script to generate XBot RT Plugin skeleton
INSTALL(PROGRAMS scripts/generate_XBot_Plugin.sh DESTINATION bin)
# install script to generate XBot RT FSM Plugin skeleton
INSTALL(PROGRAMS scripts/generate_XBot_PluginFSM.sh DESTINATION bin)
# install script to generate XBot IO Plugin skeleton
INSTALL(PROGRAMS scripts/generate_XBot_IOPlugin.sh DESTINATION bin)
