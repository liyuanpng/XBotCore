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


#include <XCM/XBotYARP/CommunicationInterfaceYARP.h>

#define WRAPPER_RATE 5
#define ANALOG_SERVER_RATE 5

namespace XBot {

CommunicationInterfaceYARP::CommunicationInterfaceYARP(): 
    CommunicationInterface()
{

}

CommunicationInterfaceYARP::CommunicationInterfaceYARP(XBotInterface::Ptr robot): 
    CommunicationInterface(robot)
{
    if(!yarp::os::Network::checkNetwork()){
        std::cout << "ERROR: yarpserver not running - run yarpserver" << std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp::os::Network::init();
    
    YARP_configuration();
}

void CommunicationInterfaceYARP::YARP_configuration()
{
    // NOTE credits to acardellino
    XBot::IXBotInit* xbot_init = nullptr;
    
    // iterate over the current robot in order to define dynamic wrappers
    for( const auto& c : _robot->getChainMap()) {
        // joints in chain
        yarp::os::Bottle joints;   
        yarp::os::Bottle joint_groups;  
        for(int i = 0; i < c.second->getJointNum(); i++) {
            joints.addInt(c.second->getJointId(i));
        }
        
        // open motion control device with chain joint map
        yarp::os::Property mc_config;
        mc_config.put("device","XBotMotionControl");    
        mc_config.put("joint_map", joints.toString()); 
        mc_config.put("chain_name", c.first);
        _motion_control_map[c.first].open(mc_config);
        
        // init
        _motion_control_map.at(c.first).view(xbot_init);
        xbot_init->init(_robot);
        
        // defining control board wrapper
        yarp::os::Property wr_config;    
        wr_config.put("device","controlboardwrapper2"); 
        wr_config.put("robot_name", "bigman"); 
        wr_config.put("name", "/" + wr_config.find("robot_name").asString() + "/" + c.first);
        wr_config.put("period", WRAPPER_RATE);           
        
        // NOTE are you crazy, Mr YARP?
        yarp::os::Bottle chains; 
        yarp::os::Bottle& actual_chain_list = chains.addList();     
        actual_chain_list.addString("networks");
        yarp::os::Bottle& actual_chain = actual_chain_list.addList();
        actual_chain.addString(c.first);
        // wrapper chains config
        wr_config.fromString(chains.toString(), false);
        
        // wrapper joints config
        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        wr_config.put(c.first, joint_groups.toString());
        wr_config.put("joints", joints.size()); 
        std::cout << "wr_config joints : "<< wr_config.toString() << std::endl;
        _wrapper_map[c.first].open(wr_config);
        
        // view on the wrapper and attach the poly driver
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = c.first;
        poly_descriptor.poly = &_motion_control_map.at(c.first);
        poly_list.push(poly_descriptor);
        
        _wrapper_map.at(c.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
    }
    
    // iterate over the actual robot FT in order to define dynamic wrappers
    for(auto& ft_j : _robot->getForceTorque()) {
        
        // open motion control device with chain joint map
        yarp::os::Property ft_config;
        ft_config.put("device","XBotFT");   
        ft_config.put("channels", 6);  
        ft_config.put("ft_name", ft_j.first); 
        _ft_map[ft_j.first].open(ft_config);
        
        // init
        _ft_map.at(ft_j.first).view(xbot_init);
        xbot_init->init(_robot);

        // defining control board wrapper
        yarp::os::Property wr_config;    
        std::string aux = ft_j.first;
//         aux = aux.substr(0, aux.size() - 6);
        
        wr_config.put("device","analogServer"); 
        wr_config.put("robot_name", "bigman"); // TBD GET FROM SOMEWHERE 
        wr_config.put("name", "/" + wr_config.find("robot_name").asString() + "/" + aux + "/" + "analog:o/forceTorque");
        wr_config.put("period", ANALOG_SERVER_RATE);            // TBD do it from config YAML
        _analog_server_map[ft_j.first].open(wr_config);
        
        // view on the wrapper and attach the poly driver
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = ft_j.first;
        poly_descriptor.poly = &_ft_map.at(ft_j.first);
        poly_list.push(poly_descriptor);
        
        _analog_server_map.at(ft_j.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
        
    }
    
    // iterate over the actual robot IMU in order to define dynamic wrappers
    for(auto& imu_j : _robot->getImu()) {
        
        // open motion control device with chain joint map
        yarp::os::Property imu_config;
        imu_config.put("device","VN100");   
//         imu_config.put("channels", 6);  
        imu_config.put("imu_name", imu_j.first); 
        _imu_map[imu_j.first].open(imu_config);
        
        // init
        _imu_map.at(imu_j.first).view(xbot_init);
        xbot_init->init(_robot);

        // defining control board wrapper
        yarp::os::Property wr_config;    
        std::string aux = imu_j.first;
//         aux = aux.substr(0, aux.size() - 6);
        
        wr_config.put("device","inertial"); 
        wr_config.put("robot_name", "bigman"); // TBD GET FROM SOMEWHERE 
        wr_config.put("name", "/" + wr_config.find("robot_name").asString() + "/" + aux + "/" + "inertial");
//         wr_config.put("period", ANALOG_SERVER_RATE);            // TBD do it from config YAML
        _inertial_map[imu_j.first].open(wr_config);
        
        // view on the wrapper and attach the poly driver
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = imu_j.first;
        poly_descriptor.poly = &_imu_map.at(imu_j.first);
        poly_list.push(poly_descriptor);
        
        _inertial_map.at(imu_j.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
        
    }
}


bool CommunicationInterfaceYARP::advertiseSwitch(const std::string& port_name)
{
    return false;
}

bool CommunicationInterfaceYARP::advertiseCmd(const std::string& port_name)
{
    return false;
}

bool CommunicationInterfaceYARP::advertiseMasterCommunicationInterface()
{
    return false;
}


bool CommunicationInterfaceYARP::receiveFromSwitch(const std::string& port_name, std::string& message)
{
    return false;
}

bool CommunicationInterfaceYARP::receiveFromCmd(const std::string& port_name, std::string& message)
{
    return false;
}

bool CommunicationInterfaceYARP::receiveMasterCommunicationInterface(std::string& framework_name)
{
    return false;
}


void CommunicationInterfaceYARP::sendRobotState()
{
    return;
}

void CommunicationInterfaceYARP::receiveReference()
{
    return;
}



}