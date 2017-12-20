/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <XCM/XBotControlPlugin.h>

namespace XBot {

XBotControlPlugin::XBotControlPlugin()
{

}

XBotControlPlugin::~XBotControlPlugin()
{

  Logger::info() << "~XBotControlPlugin" << Logger::endl();
}

XBot::Command&  XBotControlPlugin::getCmd(){
  
  return current_command;
}

void XBotControlPlugin::setCmd( XBot::Command& cmd){
  
  current_command = cmd;
}

bool XBotControlPlugin::init(   XBot::Handle::Ptr handle,
                                std::string name,
                                std::shared_ptr< PluginStatus > cstatus,                      
                                std::shared_ptr<HALInterface> halInterface,
                                std::shared_ptr< XBot::IXBotModel > model )
{
    // initialize name and interfaces
    set_xbotcore_halInterface(halInterface);
    this->name = name;
    set_xbotcore_joint(std::shared_ptr<XBot::IXBotJoint>(halInterface));
    set_xbotcore_model(model);
    set_xbotcore_ft(std::shared_ptr<XBot::IXBotFT>(halInterface));
    set_xbotcore_imu(std::shared_ptr<XBot::IXBotIMU>(halInterface));
    set_xbotcore_hand(std::shared_ptr<XBot::IXBotHand>(halInterface));

    // initialize the command port
    _custom_status = cstatus;
                     
    return init_control_plugin(handle);

}

void XBotControlPlugin::run(double time, double period)
{
    
    control_loop(time, period);
}

}
