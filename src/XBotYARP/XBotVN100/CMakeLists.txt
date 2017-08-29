
yarp_prepare_device(VN100 TYPE yarp::dev::VN100 INCLUDE VN100.hpp WRAPPER inertial)

if(ENABLE_VN100)

    message(INFO " Compiling VN_100 YARP device")

    include_directories(library/include ${CMAKE_CURRENT_SOURCE_DIR})
    include_directories(${YARP_INCLUDE_DIRS})

    yarp_add_plugin(VN100 VN100.cpp VN100.hpp)
    target_link_libraries(VN100 ${YARP_LIBRARIES})

    yarp_install(TARGETS VN100
                 COMPONENT runtime
                 LIBRARY DESTINATION ${XBOTYARP_DYNAMIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES VN100.ini
                 COMPONENT runtime
                 DESTINATION ${XBOTYARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()
    message(INFO " NOT Compiling VN100 YARP device")

endif(ENABLE_VN100)
