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


#include <XCM/XBotCommunicationInterfaceROS.h>

namespace XBot {

bool RosCommunicationInterface::callback(std_srvs::SetBoolRequest& req,
                                         std_srvs::SetBoolResponse& res,
                                         const std::string& port_name)
{
    _msgs.at(port_name) = req.data ? "start" : "stop";
    res.success = true;
    return true;
}


RosCommunicationInterface::RosCommunicationInterface():
    CommunicationInterface()
{
    int argc = 1;
    char *arg = "dummy_arg";
    char** argv = &arg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "ros_communication_interface");
    }

    _nh = std::make_shared<ros::NodeHandle>();
}

RosCommunicationInterface::RosCommunicationInterface(XBotInterface::Ptr robot):
    CommunicationInterface(robot)
{
    int argc = 1;
    char *arg = "dummy_arg";
    char** argv = &arg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "ros_communication_interface");
    }

    _nh = std::make_shared<ros::NodeHandle>();
}

void RosCommunicationInterface::sendRobotState()
{

}

void RosCommunicationInterface::receiveReference()
{

}

bool RosCommunicationInterface::advertiseSwitch(const std::string& port_name)
{
    if( _services.count(port_name) > 0 ){
        return false;
    }

    _services[port_name] = _nh->advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>
                                    (port_name,
                                     boost::bind(&RosCommunicationInterface::callback,
                                                 this,
                                                 _1, _2, port_name)
                                     );
    _msgs[port_name] = "";

    std::cout << "Advertised service " << port_name << std::endl;

    return true;
}

bool RosCommunicationInterface::receiveFromSwitch(const std::string& port_name, std::string& message)
{
    ros::spinOnce();
    
    auto it = _msgs.find(port_name);

    if( it == _msgs.end() ) return false;

    message = it->second;
    if( message != "" ){
        return true;
    }
    else {
        return false;
    }
}


}