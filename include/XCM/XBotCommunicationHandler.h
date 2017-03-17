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

#ifndef __XBOT_COMMUNICATION_HANDLER__
#define __XBOT_COMMUNICATION_HANDLER__

#include <XCM/XBotThread.h>

#include <XCM/XBotCommunicationInterface.h>
#include <XCM/XBotXDDP.h>

#include <XBotCore-interfaces/XDomainCommunication.h>


#ifdef USE_ROS_COMMUNICATION_INTERFACE
#include <XCM/CommunicationInterfaceROS.h>
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
#include <XCM/CommunicationInterfaceYARP.h>
#endif

namespace XBot
{
class CommunicationHandler : public Thread_hook
{
public:

    CommunicationHandler(std::string path_to_config);

    virtual void th_init(void*);
    virtual void th_loop(void*);

    virtual ~CommunicationHandler();

protected:

private:



    std::string _path_to_config;

    std::vector<std::string> _plugin_names;
    std::vector<std::string> _switch_names;

    std::vector<XBot::PublisherNRT<XBot::Command>> _command_pub_vector;

    XBot::XBotXDDP::Ptr _xddp_handler;
    XBot::RobotInterface::Ptr _robot;

    std::vector<XBot::CommunicationInterface::Ptr> _communication_ifc_vector;
    XBot::CommunicationInterface::Ptr _master_communication_ifc;
};
}

#endif