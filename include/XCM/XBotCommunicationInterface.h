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


#ifndef __XBOT_COMMUNICATION_INTERFACE_H__
#define __XBOT_COMMUNICATION_INTERFACE_H__

#include <XBotInterface/RobotInterface.h>
#include <XCM/XBotXDDP.h>

namespace XBot {

class CommunicationInterface {

public:

    typedef std::shared_ptr<CommunicationInterface> Ptr;

    CommunicationInterface() {}
    virtual ~CommunicationInterface() {};
    CommunicationInterface(XBotInterface::Ptr robot, 
                           XBot::XBotXDDP::Ptr xddp_handler = nullptr, 
                           XBot::IXBotJoint::Ptr xbot_joint = nullptr
                          ):
        _robot(robot),
        _xbot_joint(xbot_joint),
        _xddp_handler(xddp_handler),
        _master_communication_interface_port("MasterCommunicationInterface_switch") {}

    virtual void sendRobotState() = 0;
    virtual void receiveReference() = 0;
    virtual void resetReference() { std::cout << "TO BE IMPLEMENTED" << std::endl; }

    virtual bool advertiseSwitch(const std::string& port_name) = 0;
    virtual bool receiveFromSwitch(const std::string& port_name, std::string& message) = 0;

    virtual bool advertiseCmd(const std::string& port_name) = 0;
    virtual bool receiveFromCmd(const std::string& port_name, std::string& message) = 0;  // TBD template message

    virtual bool advertiseMasterCommunicationInterface() = 0;
    virtual bool receiveMasterCommunicationInterface(std::string& framework_name) = 0;

    virtual void advertiseStatus(const std::string& plugin_name){}
    virtual bool setPluginStatus(const std::string& plugin_name, const std::string& status){}
    virtual std::string getPluginStatus(const std::string& plugin_name){ return "TO BE IMPLEMENTED"; }

protected:

    XBotInterface::Ptr _robot;
    std::string _master_communication_interface_port;
    
    XBot::XBotXDDP::Ptr _xddp_handler;
    XBot::IXBotJoint::Ptr _xbot_joint;

private:


};

}
#endif