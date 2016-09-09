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


#ifndef __X_BOT_COMMUNICATION_HANDLER_H__
#define __X_BOT_COMMUNICATION_HANDLER_H__

#include <iit/ecat/advr/esc.h>
#include <iit/advr/thread_util.h>
#include <XBotCore/XBotEcat.h>

#include <mutex>
#include <cstring>

#include <XBotCore/IXBotJoint.h>
#include <XBotCore/IXBotFT.h>
// #include <XBotCore/IXBotChain.h>
// #include <XBotCore/IXBotRobot.h>

#include <XBotCore/XBotCoreModel.hpp>

namespace XBot
{
    class XBotCommunicationHandler;
}


/**
 * @brief XBotCore Communication Handler component: non-RT thread with ready to use
 *        non-RT XBotCore Interfaces implementation (using XDDP Pipes)
 * 
 */
class XBot::XBotCommunicationHandler : public Thread_hook,
                                       public XBot::IXBotJoint,
                                       public XBot::IXBotFT
                        
{
public:
    
    XBotCommunicationHandler(std::string config_file);
    virtual ~XBotCommunicationHandler();
    
    std::map<std::string,std::vector<int> > get_robot_map();
    std::map<std::string,int> get_ft_sensors_map();
    XBot::XBotCoreModel get_robot_model();
    
    virtual void th_init(void* );
    virtual void th_loop(void* );
    
    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, float& link_pos) final;
    
    virtual bool get_motor_pos(int joint_id, float& motor_pos) final;
    
    virtual bool get_link_vel(int joint_id, float& link_vel) final;
    
    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel) final;
    
    virtual bool get_torque(int joint_id, int16_t& torque) final;
    
    virtual bool get_max_temperature(int joint_id, uint16_t& max_temperature) final;
    
    virtual bool get_fault(int joint_id, uint16_t& fault) final;
    
    virtual bool get_rtt(int joint_id, uint16_t& rtt) final;
    
    virtual bool get_op_idx_ack(int joint_id, uint16_t& op_idx_ack) final;
    
    virtual bool get_aux(int joint_id, float& aux) final;
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const float& pos_ref) final;
    
    virtual bool set_vel_ref(int joint_id, const int16_t& vel_ref) final;
    
    virtual bool set_tor_ref(int joint_id, const int16_t& tor_ref) final;
    
    virtual bool set_gains(int joint_id, const std::vector<uint16_t>& gains) final;
    
    virtual bool set_fault_ack(int joint_id, const int16_t& fault_ack) final;
    
    virtual bool set_ts(int joint_id, const uint16_t& ts) final;
    
    virtual bool set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux) final;
    
    virtual bool set_aux(int joint_id, const float& aux) final;
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< float >& ft, int channels = 6) final;
    virtual bool get_ft_fault(int ft_id, uint16_t& fault) final;
    virtual bool get_ft_rtt(int ft_id, uint16_t& rtt) final;
    
    
    // TBD do an interface that does it
    bool get_min_pos(int joint_id, float& min_pos);
    bool get_max_pos(int joint_id, float& max_pos);
    bool get_ctrl_status_cmd(int joint_id, uint16_t& ctrl_status_cmd);


private:
        
    /**
     * @brief Robot model loaded in XBotCore
     * 
     */
    XBotCoreModel model;
        
    /**
     * @brief The complete path the the urdf file to load
     * 
     */
    std::string urdf_path;
    
    /**
     * @brief The complete path the the srdf file to load
     * 
     */
    std::string srdf_path;
    
    /**
     * @brief Joint id to joint name map configuration file
     * 
     */
    std::string joint_map_config;

    /**
     * @brief map between the chain name and the id of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<int>> robot;
    
    /**
     * @brief ft map 
     * 
     */
    std::map<std::string, int> ft;
    
    /**
     * @brief fd reading from pipes: we read the robot from XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_read;
    
    /**
     * @brief fd writing to pipes: we writhe to the robot with XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_write;
    
    /**
     * @brief fd reading from pipes: we read the robot F-T XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_ft_read;
    
    /**
     * @brief fd reading from pipes: we read the robot SDO XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_sdo_read;
    
    /**
     * @brief number of bytes read from pipes
     * 
     */
    int n_bytes;
    
    /**
     * @brief mutex
     * virtual std::vector< float > get_ft();
     */
    std::map<int,std::shared_ptr<std::mutex>> mutex;
    
    
    /**
     * @brief PDO Motor map
     * 
     */
    std::map<int, std::shared_ptr<iit::ecat::advr::McEscPdoTypes>> pdo_motor;
    
    
    /**
     * @brief SDO Info map
     * 
     */
    std::map<int, std::shared_ptr<XBot::sdo_info>> sdo_info;


    

};

#endif //__X_BOT_COMMUNICATION_HANDLER_H__
