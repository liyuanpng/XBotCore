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
#include <XCM/CommunicationInterfaceROS.h>

namespace XBot {

NRT_ROS_Subscriber::NRT_ROS_Subscriber()
{
    _ros_communication = std::make_shared<XBot::CommunicationInterfaceROS>();
}

NRT_ROS_Subscriber::NRT_ROS_Subscriber(const std::string& socket_name) : 
    NRT_ROS_Subscriber()
{
//     _ros_communication->advertiseSwitch(socket_name);
}

void NRT_ROS_Subscriber::init(const std::string& socket_name)
{

    
    if(boost::algorithm::ends_with(socket_name, "_switch")) {
        _ros_communication->advertiseSwitch(socket_name);
        _is_cmd = false;
        _is_switch = true;
    }
    else if(boost::algorithm::ends_with(socket_name, "_cmd")) {
        _ros_communication->advertiseCmd(socket_name);
        _is_cmd = true;
        _is_switch = false;
    }
    _name = socket_name;
}

bool NRT_ROS_Subscriber::read(XBot::Command& data)
{
    if(_is_switch && _ros_communication->receiveFromSwitch(_name, _aux_data)) {
        data = _aux_data;
        return true;
    }
    else if(_is_cmd && _ros_communication->receiveFromCmd(_name, _aux_data)) {
        data = _aux_data;
        return true;  
    }
    
    return false;
}


XBot::NRT_ROS_Publisher::NRT_ROS_Publisher()
{
    _ros_communication = std::make_shared<XBot::CommunicationInterfaceROS>();
}

XBot::NRT_ROS_Publisher::NRT_ROS_Publisher(const std::string& socket_name) : 
    NRT_ROS_Publisher()
{
//     _ros_communication->advertiseStatus(socket_name);
}

void XBot::NRT_ROS_Publisher::init(const std::string& socket_name)
{
    std::string aux_str = socket_name;
    aux_str.erase(aux_str.find("_status"), 7);
    _ros_communication->advertiseStatus(aux_str);
    _name = aux_str;
}

void XBot::NRT_ROS_Publisher::write(const Command& data)
{
    _ros_communication->setPluginStatus(_name, data.str());
}


}
