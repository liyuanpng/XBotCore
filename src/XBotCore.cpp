/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       Giuseppe Rigano (2017, giuseppe.rigano@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/

#include <XBotCore/XBotCore.h>
#include <boost/bind.hpp>
#include <XBotEcat.h>
//#include <Ethernet.h>

XBot::XBotCore::XBotCore(const char* config_yaml) : 
//     XBotEcat(config_yaml), 
    _path_to_config(config_yaml)
{

  //TODO use FactoryPattern
  //use shared library
  //robotInterface = new XBot::Ethernet(_path_to_config.c_str()); 
  robotInterface = new XBot::XBotEcat(_path_to_config.c_str());
}

void XBot::XBotCore::th_init( void * ){
  
  std::cout<<"INIT Thread"<<std::endl;
  robotInterface->init();
  control_init();
}

void XBot::XBotCore::th_loop( void * ){
  
  robotInterface->recv_from_slave();
  control_loop();
  robotInterface->send_to_slave();
  
}

void XBot::XBotCore::control_init(void) 
{
    
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(robotInterface);
    std::shared_ptr<XBot::IXBotFT> xbot_ft(robotInterface);
    std::shared_ptr<XBot::IXBotIMU> xbot_imu(robotInterface);
    std::shared_ptr<XBot::IXBotHand> xbot_hand(robotInterface);
    
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    
    //TODO use isRT from RobotControlInterface robotInterface.IsRt()
    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::XBotCore::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider, "XBotRTPlugins");  //"XBotRTPlugins"
    
    // define the XBotCore shared_memory for the RT plugins
    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();
    
    //
    _pluginHandler->load_plugins();
    
    //
    _pluginHandler->init_plugins(shared_memory, xbot_joint, xbot_ft, xbot_imu);
}

double XBot::XBotCore::get_time()
{
    return iit::ecat::get_time_ns() / 1e9;
}

int XBot::XBotCore::control_loop(void) 
{    
  std::cout<<"loop Thread"<<std::endl;
//     std::cout << "laurenzi" << std::endl;
    _iter++;
    _pluginHandler->run();
}

XBot::XBotCore::~XBotCore() {
    
    _pluginHandler->close();
    printf("Iteration: %d \n", _iter);
    printf("~XBotCore()\n");
}
