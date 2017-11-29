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


#ifndef __X_BOT_CORE_H__
#define __X_BOT_CORE_H__

#include <XCM/XBotUtils.h>

#include <XBotCore-interfaces/XBotPipes.h>

#include <XBotInterface/RobotInterface.h>

#include <XCM/XBotPluginHandler.h>
#include <XBotCore/HALInterface.h>
#include <XBotCore/ControllerInterface.h>
#include <XCM/Loader.h>
#include <XBotCore/XBotLoaderThread.h>

namespace XBot
{
    class XBotCore;  
}

/**
 * @brief XBotCore: RT EtherCAT thread and RT (shared-memory) XBotCore interfaces implementation.
 * 
 */
class XBot::XBotCore : public ControllerInterface
                        
{
public:
    
    XBotCore(std::string config_yaml, 
             XBot::SharedMemory::Ptr shmem, 
             Options options);
    
    XBotCore(std::string config_yaml, 
             std::shared_ptr<HALInterface> halInterface, 
             XBot::SharedMemory::Ptr shmem,
             Options options,
             std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider = nullptr
            );
             
    
    
    virtual ~XBotCore();
    
    static std::shared_ptr<Loader> getLoader();

protected:
    
    /**
     * @brief initialization function called before the control_loop
     * 
     * @param  void
     * @return void
     */
    virtual void control_init(void) final;
     
    /**
     * @brief Simply call the plugin handler loop function that will be implemented by the derived class: overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return 1 on plugin_handler_loop() success. 0 otherwise
     */
    virtual int control_loop(void) final;
    
private:    
  
    double get_time();
    
    const Options _options;
  
    std::shared_ptr<HALInterface> halInterface;
    
    std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> _time_provider;
    
    XBot::SharedMemory::Ptr _shmem;
    
    /**
     * @brief Path to YAML config file
     * 
     */
    std::string _path_to_config;
    
    // xbot robot
    XBot::RobotInterface::Ptr _robot;
    
    /**
     * @brief XBot plugin handler
     * 
     */
    XBot::PluginHandler::Ptr _pluginHandler;
    
    int _iter = 0;
  
    void init_internal();
    
    void loop_internal();

    std::string lib_file;
    
    static std::shared_ptr<Loader> loaderptr;
    
    XBot::Thread_hook* loaderth;
};

#endif //__X_BOT_CORE_H__
