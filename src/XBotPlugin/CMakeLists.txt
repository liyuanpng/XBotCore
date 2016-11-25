# CMake minimum version and policies
cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

# XBotPlugin
project(XBotPlugin)

# C++11 
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

find_package(XBotInterface REQUIRED)

# include 
include_directories(../include
                    ${XBotInterface_INCLUDE_DIRS}
                    )

# lib
add_library(XBotCommunicationPlugin SHARED
    XBotCommunicationPlugin.cpp
)

# lib
add_library(RobotInterfaceXBotRT_test SHARED
    RobotInterfaceXBotRT_test.cpp)

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(XBotCommunicationPlugin)
    set_xeno_flags(RobotInterfaceXBotRT_test)
endif()

# link the common libs
# target_link_libraries(XBotCommunicationPlugin XBotPluginHandler)
target_link_libraries(RobotInterfaceXBotRT_test ${XBotInterface_LIBRARIES})


# install lib and include
install(TARGETS XBotCommunicationPlugin RUNTIME LIBRARY DESTINATION lib)

install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")

######################################################################## RobotInterfaceXBotRT_test

set(VARS_PREFIX RobotInterfaceXBotRT_test)

set(RobotInterfaceXBotRT_test_MAJOR_VERSION 0)
set(RobotInterfaceXBotRT_test_MINOR_VERSION 0)
set(RobotInterfaceXBotRT_test_PATCH_VERSION 1)
set(RobotInterfaceXBotRT_test_VERSION ${RobotInterfaceXBotRT_test_MAJOR_VERSION}.${RobotInterfaceXBotRT_test_MINOR_VERSION}.${RobotInterfaceXBotRT_test_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(RobotInterfaceXBotRT_test PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(RobotInterfaceXBotRT_test PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})
install(TARGETS RobotInterfaceXBotRT_test
        EXPORT RobotInterfaceXBotRT_test
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)


#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS RobotInterfaceXBotRT_test)


include(InstallBasicPackageFiles)
install_basic_package_files(RobotInterfaceXBotRT_test VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    




  