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


#ifndef __XBOT_COMMUNICATION_INTERFACE_YARP_H__
#define __XBOT_COMMUNICATION_INTERFACE_YARP_H__

#include <XCM/XBotCommunicationInterface.h>
#include <XBotCore-interfaces/IXBotInit.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace XBot {

class CommunicationInterfaceYARP : public CommunicationInterface
{

public:

    CommunicationInterfaceYARP();
    CommunicationInterfaceYARP(XBotInterface::Ptr robot);
    
    CommunicationInterfaceYARP(const CommunicationInterfaceYARP&) = delete;
    
    void YARP_configuration();

    // XBot CommunicationInterface 
    
    virtual void sendRobotState();
    virtual void receiveReference();

    virtual bool advertiseSwitch(const std::string& port_name);
    virtual bool receiveFromSwitch(const std::string& port_name, std::string& message);
    
    virtual bool advertiseCmd(const std::string& port_name);
    virtual bool receiveFromCmd(const std::string& port_name, std::string& message);  // TBD template message
    
    virtual bool advertiseMasterCommunicationInterface();
    virtual bool receiveMasterCommunicationInterface(std::string& framework_name);
    
    virtual ~CommunicationInterfaceYARP() { std::cerr << "~CommunicationInterfaceYARP()" << std::endl; }

protected:

private:

    std::map<std::string, yarp::dev::PolyDriver > _motion_control_map;
    std::map<std::string, yarp::dev::PolyDriver > _wrapper_map;
    std::map<std::string, yarp::dev::PolyDriver > _ft_map;
    std::map<std::string, yarp::dev::PolyDriver > _analog_server_map;
    std::map<std::string, yarp::dev::PolyDriver > _imu_map;
    std::map<std::string, yarp::dev::PolyDriver > _inertial_map;

};

}

#endif //__XBOT_COMMUNICATION_INTERFACE_YARP_H__