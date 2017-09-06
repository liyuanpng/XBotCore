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

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

# find packages 
find_package(XBotCoreModel REQUIRED)
find_package(XCM REQUIRED)

# find_package(soem${LIB_TYPE} REQUIRED)
# find_package(iitecat_iface${LIB_TYPE} REQUIRED)
# find_package(walkman_master${LIB_TYPE} REQUIRED)
# find_package(ecat_test_utils${LIB_TYPE} REQUIRED)
find_package(sharedlibpp REQUIRED)

# communication packages
# # find_package(YARP REQUIRED)

set(COMMON_LIBS
    pthread
    jsoncpp
    yaml-cpp
)


# include 
include_directories(include ${XBotCoreModel_INCLUDE_DIRS} 
                            ${kdl_parser_INCLUDE_DIRS}
                            ${soem${LIB_TYPE}_INCLUDE_DIRS}
                            ${iitecat_iface${LIB_TYPE}_INCLUDE_DIRS}
                            ${walkman_master${LIB_TYPE}_INCLUDE_DIRS}
                            ${ecat_test_utils${LIB_TYPE}_INCLUDE_DIRS}
                            ${sharedlibpp_INCLUDE_DIRS}
                            ${XCM_INCLUDE_DIRS}
                            include/XBotCore/
                            ${CMAKE_CURRENT_SOURCE_DIR}/../FRI_Remote

                    )
                            

add_definitions(-DHAVE_SOCKLEN_T)                            
# exe
add_executable(${PROJECT_NAME}
    src/XBotMain.cpp
)

# lib
add_library(XBotCoreThread SHARED
#     src/XBotEcat.cpp
    src/XBotCore.cpp
#     src/ec_boards_base.cpp
#     src/Ethernet.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../FRI_Remote/friudp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../FRI_Remote/friremote.cpp
    src/Kuka.cpp
)


# set Xenomain flags if Xenomai is founded
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
    set_xeno_flags(XBotCoreThread)
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS}
                                      ${XBotCoreModel_LIBRARIES}
                                      ${soem${LIB_TYPE}_LIBRARIES} 
                                      ${iitecat_iface${LIB_TYPE}_LIBRARIES}
                                      ${walkman_masterwalkman_master${LIB_TYPE}_LIBRARIES}
                                      ${ecat_test_utils${LIB_TYPE}_LIBRARIES}
                                      XBotCoreThread
                                      )

                                      
                                      
target_link_libraries(XBotCoreThread    ${COMMON_LIBS}
                                        ${XBotCoreModel_LIBRARIES}
                                        ${soem${LIB_TYPE}_LIBRARIES} 
                                        ${iitecat_iface${LIB_TYPE}_LIBRARIES}
                                        ${walkman_master${LIB_TYPE}_LIBRARIES}
                                        ${ecat_test_utils${LIB_TYPE}_LIBRARIES}
                                        ${sharedlibpp_LIBRARIES}
                                        ${XCM_LIBRARIES}
                                        )                                      

install(TARGETS XBotCoreThread RUNTIME LIBRARY DESTINATION lib)

# documentation
add_subdirectory(doc)

########################################################################

set(VARS_PREFIX ${PROJECT_NAME})

set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 1)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

include(AddUninstallTarget)

if(NOT Xenomai_FOUND)
    # NOTE do it manually on the install
    #set_cap_install(${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
endif()


