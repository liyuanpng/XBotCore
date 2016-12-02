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

    // TBD can be auto-generated based on the RX PDO
    // NOTE getters
    virtual bool get_link_pos(int joint_id, float& link_pos) = 0;
    
    virtual bool get_motor_pos(int joint_id, float& motor_pos) = 0;
    
    virtual bool get_link_vel(int joint_id, int16_t& link_vel) = 0;
    
    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel) = 0;
    
    virtual bool get_torque(int joint_id, float& torque) = 0;
    
    virtual bool get_temperature(int joint_id, uint16_t& temperature) = 0;
    
    virtual bool get_fault(int joint_id, uint16_t& fault) = 0;
    
    virtual bool get_rtt(int joint_id, uint16_t& rtt) = 0;
    
    virtual bool get_op_idx_ack(int joint_id, uint16_t& op_idx_ack) = 0;
    
    virtual bool get_aux(int joint_id, float& aux) = 0;
    
    // TBD can be auto-generated based on the TX PDO
    // NOTE setters
    virtual bool set_pos_ref(int joint_id, const float& pos_ref) = 0;
    
    virtual bool set_vel_ref(int joint_id, const int16_t& vel_ref) = 0;
    
    virtual bool set_tor_ref(int joint_id, const int16_t& tor_ref) = 0;
    
    virtual bool set_gains(int joint_id, const std::vector<uint16_t>&) = 0;
    
    virtual bool set_fault_ack(int joint_id, const int16_t& fault_ack) = 0;
    
    virtual bool set_ts(int joint_id, const uint16_t& ts) = 0;
    
    virtual bool set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux) = 0;
    
    virtual bool set_aux(int joint_id, const float& aux) = 0;
    
    
    virtual ~IXBotJoint() {
        printf("~IXBotJoint()\n");
    };
};

#endif //__I_X_BOT_JOINT_H__
