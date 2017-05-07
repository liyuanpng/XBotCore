# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alberto Cardellino
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_device(XBotMotionControl TYPE XBot::dev::XBotMotionControl INCLUDE XCM/XBotYARP/XBotMotionControl.h)

IF (ENABLE_XBotMotionControl)

  INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${XBotYarp_SOURCE_DIR}/include/)
  
  yarp_add_plugin(XBotMotionControl XBotMotionControl.cpp )
  TARGET_LINK_LIBRARIES(XBotMotionControl ${YARP_LIBRARIES})
  
  yarp_install(TARGETS XBotMotionControl
               COMPONENT runtime
               LIBRARY DESTINATION ${XBOTYARP_DYNAMIC_PLUGINS_INSTALL_DIR})
               
  yarp_install(FILES XBotMotionControl.ini
               COMPONENT runtime
               DESTINATION ${XBOTYARP_PLUGIN_MANIFESTS_INSTALL_DIR})
               
ENDIF (ENABLE_XBotMotionControl)


