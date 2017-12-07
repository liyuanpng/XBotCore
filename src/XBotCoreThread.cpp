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

#include <XBotCore/XBotCoreThread.h>
#include <XBotCore/HALInterfaceFactory.h>

XBot::XBotCoreThread::XBotCoreThread(std::string config_yaml, 
                   XBot::SharedMemory::Ptr shared_memory,  
                   XBot::Options options, 
                   HALInterface::Ptr hal,
                   std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider)   
{
    int period = 1;
    if(options.xbotcore_dummy_mode){      
         period = options.xbotcore_period_us;
    }
      
    set_thread_name("XBOT");
    
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    // obtain hal
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
    std::string lib_file = "";
    std::string lib_name="";
    
    HALInterface::Ptr __hal;
    
    if(options.xbotcore_dummy_mode){ // dummy mode
    
        lib_file = "libXBotDummy";
        lib_name = "DUMMY";
        
        // NOTE dummy needs high level config
        abs_low_level_config = std::string(config_yaml);
        
        __hal = HALInterfaceFactory::getFactory(lib_file, lib_name, abs_low_level_config.c_str());
        
    }
    else if(hal) // hal provided by constructor
    {
        __hal = hal;
    }
    else if(!hal && hal_lib) // hal is not provided by the constructor
    {
        lib_file = hal_lib["lib_file"].as<std::string>();
        lib_name = hal_lib["lib_name"].as<std::string>();
        
        __hal = HALInterfaceFactory::getFactory(lib_file, lib_name, abs_low_level_config.c_str());
    }
    else{
        throw std::runtime_error("Unable to load HAL");
    }
    

    controller = std::shared_ptr<ControllerInterface>(new XBot::XBotCore(config_yaml, 
                                                                         __hal, 
                                                                         shared_memory, 
                                                                         options, 
                                                                         time_provider)
                                                     );
}

void XBot::XBotCoreThread::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotCoreThread::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotCoreThread::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotCoreThread::set_thread_priority()
{

    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotCoreThread::th_init( void * ){
  
    controller->control_init();
}

void XBot::XBotCoreThread::th_loop( void * ){  
  
    controller->control_loop();  
}

XBot::XBotCoreThread::~XBotCoreThread() {
    
   Logger::info() << "~XBotCoreThread()" << Logger::endl();
}
