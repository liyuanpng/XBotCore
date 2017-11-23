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


#ifndef __I_X_BOT_JOINT_H__
#define __I_X_BOT_JOINT_H__

#include <map>
#include <vector>
#include <memory>
#include <string>

#include <XBotInterface/RtLog.hpp>

namespace XBot
{
    class IXBotJoint;
}

/**
 * @brief XBotCore Joint Interface
 * 
 */
class XBot::IXBotJoint
{

public:  
    
    typedef std::shared_ptr<IXBotJoint> Ptr;

    // TBD can be auto-generated based on the RX PDO
    // NOTE RX getters
    virtual bool get_link_pos(int joint_id, double& link_pos) = 0;
    
    virtual bool get_motor_pos(int joint_id, double& motor_pos) = 0;
    
    virtual bool get_link_vel(int joint_id, double& link_vel) = 0;
    
    virtual bool get_motor_vel(int joint_id, double& motor_vel) = 0;
    
    virtual bool get_torque(int joint_id, double& torque) = 0;
    
    virtual bool get_temperature(int joint_id, double& temperature) = 0;
    
    virtual bool get_gains(int joint_id, std::vector<double>& gain_vector) = 0;
    
    virtual bool get_fault(int joint_id, double& fault) = 0;
    
    virtual bool get_rtt(int joint_id, double& rtt) = 0;
    
    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) = 0;
    
    virtual bool get_aux(int joint_id, double& aux) = 0;
    
    // NOTE TX getters
    virtual bool get_pos_ref(int joint_id, double& pos_ref) {};
    
    virtual bool get_vel_ref(int joint_id, double& vel_ref) {};
    
    virtual bool get_tor_ref(int joint_id, double& tor_ref) {};
    
    // TBD can be auto-generated based on the TX PDO
    // NOTE setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) = 0;
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) = 0;
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) = 0;
    
    virtual bool set_gains(int joint_id, const std::vector<double>&) = 0;
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) = 0;
    
    virtual bool set_ts(int joint_id, const double& ts) = 0;
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) = 0;
    
    virtual bool set_aux(int joint_id, const double& aux) = 0;
    
    
    virtual ~IXBotJoint() {
        if(Logger::GetVerbosityLevel() == Logger::Severity::LOW)
            std::cout << __func__ << std::endl;
    };
};

#endif //__I_X_BOT_JOINT_H__
