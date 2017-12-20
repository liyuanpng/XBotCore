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

# XBotCore
project(XBotCore)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)
include(MacroYCMInstallLibrary)
include(ExternalProject)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-stack-protector")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable Xenomai flag and if enabled find required packages
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

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

# find packages 
# find first YCM in order to have ${YCM_MODULE_PATH} defined for 3rd party software like Eigen3
find_package(YCM REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(XBotCoreModel REQUIRED)
find_package(catkin QUIET COMPONENTS roscpp sensor_msgs std_srvs message_generation robot_state_publisher)
find_package(YARP QUIET)
find_package(kdl_parser REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(sharedlibpp REQUIRED)

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


    add_dependencies(AdvrCommandMessage XBotCore_generate_messages_cpp)
    add_dependencies(AdvrJointStateMessage XBotCore_generate_messages_cpp)

endif()


# communication packages
# # find_package(YARP REQUIRED)

set(COMMON_LIBS
    pthread
    yaml-cpp
)


include_directories(include ${XBotInterface_INCLUDE_DIRS}
                            ${XBotCore-interfaces_INCLUDE_DIRS}
                            ${XBotCoreModel_INCLUDE_DIRS} 
                            ${sharedlibpp_INCLUDE_DIRS}
                            ${kdl_parser_INCLUDE_DIRS}
                            include/XBotCore/
                            include/Civet/
                            include/Web/
                            )


# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XCM_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)


set(COMMUNICATION_INTERFACE_SOURCE_FILES)
# set(COMMUNICATION_INTERFACE_LIBRARIES)

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

#webserver
add_library(webserver SHARED
                        src/CommunicationInterfaceWebServer.cpp
                        src/Civet/CivetServer.cpp
                        src/Civet/civetweb.c
                        src/Web/HttpCivetHandler.cpp
                        src/Web/HttpHandler.cpp
                        src/Web/SharedData.cpp
                        src/Web/WebRobotState.cpp
                        src/Web/WebSocketCivetHandler.cpp
                        )

add_library(XCM SHARED ${XBotInterface_INCLUDES}
                        src/XDomainCommunication.cpp
                        src/XBotPlugin.cpp
                        src/XBotControlPlugin.cpp
                        src/XBotPluginHandler.cpp
#                         src/XBotModuleHandler.cpp
                        src/XBotControlModule.cpp
                        src/XBotCommunicationHandler.cpp
                        src/XBotXDDP.cpp
                        src/XBotESCUtils.cpp
                        src/IOPlugin.cpp
                        src/XBotPluginStatus.cpp    
                        src/XBotCommunicationInterfaceFactory.cpp
                        src/PluginFactory.cpp
                        src/IOPluginFactory.cpp
                        ${COMMUNICATION_INTERFACE_SOURCE_FILES}
                        ${RT_NRT_SRC}  
                        src/Loader.cpp
                        )
                        
add_library(XCM_no_xeno SHARED ${XBotInterface_INCLUDES}
                        src/XDomainCommunication.cpp
                        src/XBotPlugin.cpp
                        src/XBotControlPlugin.cpp
                        src/XBotPluginHandler.cpp
#                         src/XBotModuleHandler.cpp
                        src/XBotControlModule.cpp
                        src/XBotCommunicationHandler.cpp
                        src/XBotXDDP.cpp
                        src/XBotESCUtils.cpp
                        src/IOPlugin.cpp
                        src/XBotPluginStatus.cpp 
                        src/XBotCommunicationInterfaceFactory.cpp
                        src/PluginFactory.cpp
                        src/IOPluginFactory.cpp
                        ${COMMUNICATION_INTERFACE_SOURCE_FILES}       
                        src/XBotNRT_thread.cpp
                        src/Loader.cpp
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

add_executable(${PROJECT_NAME}
    src/XBotMain.cpp    
)

# lib
add_library(XBotCoreThread SHARED
    src/XBotCore.cpp
    src/XBotCoreThread.cpp
    src/HALInterfaceFactory.cpp
    src/signal_handler.cpp
    src/XBotLoaderThread.cpp
)

if ( Xenomai_FOUND )
    set_xeno_flags(XCM)
    set_xeno_flags(${PROJECT_NAME})
    set_xeno_flags(XBotCoreThread)
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS}
                                      ${XBotCoreModel_LIBRARIES}                                      
                                      XBotCoreThread
                                      ${Boost_LIBRARIES}
                                      )

                                      
                                      
target_link_libraries(XBotCoreThread    ${COMMON_LIBS}
                                        ${XBotCoreModel_LIBRARIES}                                        
                                        ${sharedlibpp_LIBRARIES}
                                        XCM
                                        ) 
                                        
# documentation
add_subdirectory(doc)

ycm_library_install(XBotCoreThread 1 0 0)
ycm_library_install(XBotCore 1 0 0)

add_executable(DummyMain src/DummyMain.cpp)
target_link_libraries(DummyMain XCM ${Boost_LIBRARIES})
# target_link_libraries(DummyMain webserver )
ycm_library_install(DummyMain 1 0 0)

add_executable(CommunicationHandler src/XBotCommunicationMain.cpp)
target_link_libraries(CommunicationHandler XCM ${COMMUNICATION_INTERFACE_LIBRARIES})
# target_link_libraries(CommunicationHandler webserver )
ycm_library_install(CommunicationHandler 1 0 0)

add_executable(NRTDeployer src/NRTDeployer.cpp src/NRTDeployerMain.cpp)
target_link_libraries(NRTDeployer XCM_no_xeno ${Boost_LIBRARIES})
# target_link_libraries(NRTDeployer webserver )
ycm_library_install(NRTDeployer 1 0 0)


add_executable(RobotStateLogger src/RobotStateLogger.cpp)
target_link_libraries(RobotStateLogger XCM ${COMMUNICATION_INTERFACE_LIBRARIES})
ycm_library_install(RobotStateLogger 1 0 0)

add_subdirectory(src/XBotPlugin)


ycm_library_install(webserver 1 0 0) 

########################################################################
ycm_library_install(XCM 1 0 0)                                    
########################################################################
ycm_library_install(XCM_no_xeno 1 0 0) 
########################################################################                                    

if( ${catkin_FOUND} )
########################################################################
ycm_library_install(AdvrCommandMessage 1 0 0)
ycm_library_install(AdvrJointStateMessage 1 0 0)
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