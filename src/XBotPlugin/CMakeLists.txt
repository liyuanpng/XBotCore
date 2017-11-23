# CMake minimum version and policies
cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

# XBotPlugin
project(XBotPlugin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

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
                   )

# lib
add_library(XBotCommunicationPlugin SHARED
    XBotCommunicationPlugin.cpp
)

# lib
add_library(XBotLoggingPlugin SHARED
    XBotLoggingPlugin.cpp
)

# lib
add_library(XBotNRTRef SHARED
    XBotNRTRef.cpp
)

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(XBotCommunicationPlugin)
    set_xeno_flags(XBotNRTRef)
endif()

# linking
target_link_libraries(XBotCommunicationPlugin XCM)
target_link_libraries(XBotLoggingPlugin XCM)
target_link_libraries(XBotNRTRef XCM)

# install include

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")
 
ycm_library_install(XBotCommunicationPlugin 1 0 0)

ycm_library_install(XBotLoggingPlugin 1 0 0)       

ycm_library_install(XBotNRTRef 1 0 0)