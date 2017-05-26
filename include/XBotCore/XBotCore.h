/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
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
*/


#ifndef __X_BOT_CORE_H__
#define __X_BOT_CORE_H__

#include <iit/ecat/advr/esc.h>

#include <XBotCore/XBotEcat.h>

#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotFT.h>
#include <XBotCore-interfaces/XBotPipes.h>

#include <XBotInterface/RobotInterface.h>

#include <XCM/XBotPluginHandler.h>

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
class XBot::XBotCore : public   XBot::XBotEcat,
                       public   XBot::IXBotJoint,
                       public   XBot::IXBotFT,
                       public   XBot::IXBotIMU
                        
{
public:
    
    XBotCore(const char * config_yaml);
    virtual ~XBotCore();
    
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
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) final;
    virtual bool get_ft_fault(int ft_id, double& fault) final;
    virtual bool get_ft_rtt(int ft_id, double& rtt) final;
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion);
    virtual bool get_imu_fault(int imu_id, double& fault);
    virtual bool get_imu_rtt(int imu_id, double& rtt);

    
    double get_time();

private:
     
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
    
    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) final;
    
    virtual bool get_motor_pos(int joint_id, double& motor_pos) final;
    
    virtual bool get_link_vel(int joint_id, double& link_vel) final;
    
    virtual bool get_motor_vel(int joint_id, double& motor_vel) final;
    
    virtual bool get_torque(int joint_id, double& torque) final;
    
    virtual bool get_temperature(int joint_id, double& temperature) final;
    
    virtual bool get_fault(int joint_id, double& fault) final;
    
    virtual bool get_rtt(int joint_id, double& rtt) final;
    
    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) final;
    
    virtual bool get_aux(int joint_id, double& aux) final;
    
    virtual bool get_gains(int joint_id, std::vector< double >& gain_vector) final;
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) final;
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) final;
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) final;
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) final;
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) final;
    
    virtual bool set_ts(int joint_id, const double& ts) final;
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) final;
    
    virtual bool set_aux(int joint_id, const double& aux) final;

    

};

#endif //__X_BOT_CORE_H__
