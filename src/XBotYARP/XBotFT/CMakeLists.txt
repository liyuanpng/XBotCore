# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alberto Cardellino
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_device(XBotFT TYPE XBot::dev::XBotFT INCLUDE XCM/XBotYARP/XBotFT.h)

IF (ENABLE_XBotFT)

  INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${XBotYarp_SOURCE_DIR}/include/)
  
  yarp_add_plugin(XBotFT XBotFT.cpp )
  TARGET_LINK_LIBRARIES(XBotFT ${YARP_LIBRARIES})
  
  yarp_install(TARGETS XBotFT
               COMPONENT runtime
               LIBRARY DESTINATION ${XBOTYARP_DYNAMIC_PLUGINS_INSTALL_DIR})
               
  yarp_install(FILES XBotFT.ini
               COMPONENT runtime
               DESTINATION ${XBOTYARP_PLUGIN_MANIFESTS_INSTALL_DIR})
               
ENDIF (ENABLE_XBotFT)


