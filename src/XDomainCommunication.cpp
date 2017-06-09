/*
 * Copyright (C) 2017 IIT-ADVR
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


#include <XBotCore-interfaces/XDomainCommunication.h>

namespace XBot {

NRT_ROS_Subscriber::NRT_ROS_Subscriber()
{
    _ros_communication = std::make_shared<XBot::CommunicationInterfaceROS>();
}

NRT_ROS_Subscriber::NRT_ROS_Subscriber(const std::string& socket_name) : 
    NRT_ROS_Subscriber()
{
    _ros_communication->advertiseSwitch(socket_name);
}

void NRT_ROS_Subscriber::init(const std::string& socket_name)
{
    _ros_communication->advertiseSwitch(socket_name);
    _name = socket_name;
}

bool NRT_ROS_Subscriber::read(XBot::Command& data)
{
    if(_ros_communication->receiveFromSwitch(_name, _aux_data)) {
        data = _aux_data;
        return true;
    }
    
    return false;
}

}