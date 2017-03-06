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

#find_package(XBotInterface REQUIRED)

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
endif()

# link the common libs
# target_link_libraries(XBotCommunicationPlugin XBotPluginHandler)


# install lib and include
install(TARGETS XBotCommunicationPlugin RUNTIME LIBRARY DESTINATION lib)

install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")
