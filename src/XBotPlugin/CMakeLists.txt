# CMake minimum version and policies
cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

# XBotPlugin
project(XBotPlugin)

find_package(XBotCore-interfaces REQUIRED)

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

# include
include_directories(../include
                    ${XBotCore-interfaces_INCLUDE_DIRS}
                    )

# lib
add_library(XBotCommunicationPlugin SHARED
    XBotCommunicationPlugin.cpp
)

# lib
add_library(XBotLoggingPlugin SHARED
    XBotLoggingPlugin.cpp
)

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(XBotCommunicationPlugin)
endif()

# linking
target_link_libraries(XBotCommunicationPlugin XCM)
target_link_libraries(XBotLoggingPlugin XCM)


# install include)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")

######################################################################## XBotCommunicationPlugin

set(VARS_PREFIX XBotCommunicationPlugin)

set(XBotCommunicationPlugin_MAJOR_VERSION 0)
set(XBotCommunicationPlugin_MINOR_VERSION 0)
set(XBotCommunicationPlugin_PATCH_VERSION 1)
set(XBotCommunicationPlugin_VERSION ${XBotCommunicationPlugin_MAJOR_VERSION}.${XBotCommunicationPlugin_MINOR_VERSION}.${XBotCommunicationPlugin_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XBotCommunicationPlugin PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XBotCommunicationPlugin PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})
install(TARGETS XBotCommunicationPlugin
        EXPORT XBotCommunicationPlugin
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)


#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XBotCommunicationPlugin)


include(InstallBasicPackageFiles)
install_basic_package_files(XBotCommunicationPlugin VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
######################################################################## XBotLoggingPlugin

set(VARS_PREFIX XBotLoggingPlugin)

set(XBotLoggingPlugin_MAJOR_VERSION 0)
set(XBotLoggingPlugin_MINOR_VERSION 0)
set(XBotLoggingPlugin_PATCH_VERSION 1)
set(XBotLoggingPlugin_VERSION ${XBotLoggingPlugin_MAJOR_VERSION}.${XBotLoggingPlugin_MINOR_VERSION}.${XBotLoggingPlugin_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XBotLoggingPlugin PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XBotLoggingPlugin PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})
install(TARGETS XBotLoggingPlugin
        EXPORT XBotLoggingPlugin
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)


#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XBotLoggingPlugin)


include(InstallBasicPackageFiles)
install_basic_package_files(XBotLoggingPlugin VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    
