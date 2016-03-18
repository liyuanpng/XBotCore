# CMake minimum version and policies
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

# Enable Xenomai flag and if enabled find required packages
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()

# find packages 
find_package(srdfdom REQUIRED)
find_package(urdfdom REQUIRED)
find_package(kdl_parser REQUIRED)

# Common libs TBD: make it clean!
set(COMMON_LIBS
    shared_utils
    walkman_master
    iitecat_iface
    soem
    pthread
    zmq
    jsoncpp
    yaml-cpp
)



# include 
include_directories(include ${srdfdom_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})

# exe
add_executable(${PROJECT_NAME}
    src/XBotEcat.cpp
    src/XBotCore.cpp
    src/XBotPluginHandler.cpp
    src/XBotMain.cpp
)

# set Xenomain flags if Xenomai is founded
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

# link the common libs
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS}
                                      ${srdfdom_LIBRARIES}
                                      ${urdfdom_LIBRARIES}
                                      ${kdl_parser_LIBRARIES})

if(NOT Xenomai_FOUND)
    set_cap_build(${PROJECT_NAME})
endif()

# install exe and include
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")

if(NOT Xenomai_FOUND)
    set_cap_install(${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME})
endif()


# Enable srdf generation from xacro
set(ENABLE_SRDF_FROM_XACRO_GENERATION CACHE BOOL "ON")
if ( ${ENABLE_SRDF_FROM_XACRO_GENERATION} )
    # xacro
    find_package(xacro REQUIRED)
    # iterate over xacro files
    file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/configs/srdf/*.xacro)
    foreach(it ${xacro_files})
        # remove .xacro extension
        string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
        set(output_filename ${CMAKE_MATCH_1})

        # create a rule to generate ${output_filename} from {it}
        xacro_add_xacro_file(${it} ${output_filename})

        list(APPEND srdf_files ${output_filename})
    endforeach(it)

    # add an abstract target to actually trigger the builds
    add_custom_target(media_files ALL DEPENDS ${srdf_files})
endif()