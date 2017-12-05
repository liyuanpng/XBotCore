/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       Arturo Laurenzi (2016-, arturo.laurenzi@iit.it)
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
 * @author Arturo Laurenzi (2016-, arturo.laurenzi@iit.it)
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/


#include <boost/bind.hpp>
#include <XBotCore/XBotCore.h>
#include <XBotCore/HALInterfaceFactory.h>


#include <XBotInterface/Utils.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

std::shared_ptr<Loader> XBot::XBotCore::loaderptr;

XBot::XBotCore::XBotCore(std::string config_yaml, 
                         XBot::SharedMemory::Ptr shmem,  
                         Options options) : 
    _path_to_config(config_yaml),
    _shmem(shmem),
    _options(options)
{        
   
    YAML::Node root_cfg = YAML::LoadFile(config_yaml);
    
    YAML::Node x_bot_core, root;
    
    std::string abs_low_level_config = "";
    
    // check gains in XBotCore node specifing config path YAML
    if(root_cfg["XBotCore"]) {
        x_bot_core = root_cfg["XBotCore"];
            
        if(x_bot_core["config_path"]) {
            
            abs_low_level_config = XBot::Utils::computeAbsolutePath(x_bot_core["config_path"].as<std::string>());
            
            Logger::info() << "Path to low level config is " << x_bot_core["config_path"].as<std::string>() << Logger::endl();
            
            Logger::info() << "Abs path to low level config is " << abs_low_level_config << Logger::endl();
            
            root = YAML::LoadFile(abs_low_level_config);
        }
    }

    
    const YAML::Node &hal_lib = root["HALInterface"];
    
    lib_file = "";
    std::string lib_name="";
    if( hal_lib == nullptr){
      
      Logger::error() <<"HALInterface parameter missing in config file " << Logger::endl();
      exit(1);
      
    }else{
      
      lib_file = hal_lib["lib_file"].as<std::string>();
      lib_name = hal_lib["lib_name"].as<std::string>();
    }
    
    if(options.xbotcore_dummy_mode){
      
            lib_file = "libXBotDummy";
            lib_name = "DUMMY";
            
            // NOTE dummy needs high level config
            abs_low_level_config = std::string(config_yaml);
    }
    
    halInterface = HALInterfaceFactory::getFactory(lib_file, lib_name, abs_low_level_config.c_str());
    
    if(!halInterface) exit(1);
}

XBot::XBotCore::XBotCore(std::string config_yaml, 
                         std::shared_ptr<HALInterface> halinterface, 
                         XBot::SharedMemory::Ptr shmem,
                         Options options,
                         std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider
                         
                        ) : 
    _path_to_config(config_yaml),
    _shmem(shmem),
    _options(options)
{        
    _time_provider = time_provider;
    halInterface = halinterface;
    if(!halInterface) exit(1);
}

std::shared_ptr<Loader> XBot::XBotCore::getLoader(){
  
  return loaderptr;  
}

void XBot::XBotCore::init_internal()
{
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<HALInterface> hal(halInterface);
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(halInterface);
    std::shared_ptr<XBot::IXBotFT> xbot_ft(halInterface);
    std::shared_ptr<XBot::IXBotIMU> xbot_imu(halInterface);
    std::shared_ptr<XBot::IXBotHand> xbot_hand(halInterface);
    
    // TODO initilize it somewhere else
    bool xbot_enable_transmission = true;
    
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    (*anymap)["EnableTransmissionPlugins"] = boost::any(xbot_enable_transmission);
    
    //TODO use isRT from RobotControlInterface robotInterface.IsRt()
    _robot = XBot::RobotInterface::getRobot(_path_to_config, "", anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::XBotCore::get_time, this);
    
    if (!_time_provider){
      // create time provider
      _time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    }
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, _time_provider, _shmem, _options);

    _pluginHandler->load_plugins();
    
    _pluginHandler->init_plugins(hal);
    
    loaderptr = std::make_shared<Loader>(_pluginHandler);
    loaderth = new XBot::XBotLoaderThread();
    loaderth->create(false, 2);
    
    return;
}

void XBot::XBotCore::control_init(void) 
{
     halInterface->init();
     init_internal();    
     
     return;
}

double XBot::XBotCore::get_time()
{
    return XBot::get_time_ns() / 1e9;
}

int XBot::XBotCore::control_loop(void) 
{       
    int state = halInterface->recv_from_slave();
    if(state == 0)
      loop_internal();  
    halInterface->send_to_slave();
}

void XBot::XBotCore::loop_internal()
{
    _iter++;
    _pluginHandler->run();  
}

XBot::XBotCore::~XBotCore() {
    
    _pluginHandler->close();
    
    Logger::info() << "~XBotCore()" << Logger::endl();
    
    loaderth->stop();
    loaderth->join();
    delete loaderth;
}
