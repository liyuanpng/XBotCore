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

#include <iit/ecat/advr/esc.h>

// #include <XBotCore/XBotEcat.h>
#include <XBotCore-interfaces/XBotPipes.h>

#include <XBotInterface/RobotInterface.h>

#include <XCM/XBotPluginHandler.h>
#include <XCM/XBotThread.h>
#include <RobotControlInterface.h>

namespace XBot
{
    class XBotCore;
    struct XBotConversion;
}

struct XBot::XBotConversion {
    
    // RX
    double link_pos    = 1;
    double motor_pos   = 1;
    double link_vel    = 0.001;
    double motor_vel   = 0.001;
    double torque      = 1;
    double temperature = 1;
    double fault       = 1;
    double rtt         = 1;
    double op_idx_ack  = 1;
    
    // TX
    double pos_ref     = 1;
    double vel_ref     = 1;
    double tor_ref     = 100;
    double gains       = 1;
    double fault_ack   = 1;
    double ts          = 1;
    double op_idx_aux  = 1;
    
    // RX/TX
    double aux         = 1;
    
};

/**
 * @brief XBotCore: RT EtherCAT thread and RT (shared-memory) XBotCore interfaces implementation.
 * 
 */
class XBot::XBotCore : public Thread_hook
                        
{
public:
    
    XBotCore(const char * config_yaml);
    virtual ~XBotCore();
    
    
    virtual void th_init ( void * );
    virtual void th_loop ( void * );
    
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
    
    double get_time();

private:
  
    RobotControlInterface* robotControlInterface;
     
    /**
     * @brief Path to YAML config file
     * 
     */
    std::string _path_to_config;
    
    /**
     * @brief aux buffer for the last TX PDO
     * 
     */
    iit::ecat::advr::McEscPdoTypes::pdo_tx last_pdo_tx;
    
    // xbot robot
    XBot::RobotInterface::Ptr _robot;
    
    /**
     * @brief XBot plugin handler
     * 
     */
    XBot::PluginHandler::Ptr _pluginHandler;
    
    XBot::XBotConversion _conversion;
    
    int _iter = 0;

};

#endif //__X_BOT_CORE_H__
