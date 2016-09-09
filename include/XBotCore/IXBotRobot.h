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


#ifndef __I_X_BOT_ROBOT_H__
#define __I_X_BOT_ROBOT_H__

#include <map>
#include <string>

namespace XBot
{
    class IXBotRobot;
}

/**
 * @brief XBotCore Robot Interface
 * 
 */
class XBot::IXBotRobot
{
    
public:   

    // TBD can be auto-generated based on the RX PDO
    // NOTE getters
    virtual bool get_robot_link_pos(std::map<std::string, float>& link_pos) = 0;
    virtual bool get_robot_link_pos(std::map<int, float>& link_pos) = 0;
    
    virtual bool get_robot_motor_pos(std::map<std::string, float>& motor_pos) = 0;
    virtual bool get_robot_motor_pos(std::map<int, float>& motor_pos) = 0;
    
    virtual bool get_robot_link_vel(std::map<std::string, float>& link_vel) = 0;
    virtual bool get_robot_link_vel(std::map<int, float>& link_vel) = 0;
    
    virtual bool get_robot_motor_vel(std::map<std::string, int16_t>& motor_vel) = 0;
    virtual bool get_robot_motor_vel(std::map<int, int16_t>& motor_vel) = 0;
    
    virtual bool get_robot_torque(std::map<std::string, int16_t>& torque) = 0;
    virtual bool get_robot_torque(std::map<int, int16_t>& torque) = 0;
    
    virtual bool get_robot_max_temperature(std::map<int, uint16_t>& max_temperature) = 0;    
    virtual bool get_robot_max_temperature(std::map<std::string, uint16_t>& max_temperature) = 0;
    
    virtual bool get_robot_fault(std::map<int, uint16_t>& fault) = 0;    
    virtual bool get_robot_fault(std::map<std::string, uint16_t>& fault) = 0;

    virtual bool get_robot_rtt(std::map<int, uint16_t>& rtt) = 0;    
    virtual bool get_robot_rtt(std::map<std::string, uint16_t>& rtt) = 0;
    
    virtual bool get_robot_op_idx_ack(std::map<int, uint16_t>& op_idx_ack) = 0;    
    virtual bool get_robot_op_idx_ack(std::map<std::string, uint16_t>& op_idx_ack) = 0;
    
    virtual bool get_robot_aux(std::map<std::string, float>& aux) = 0;
    virtual bool get_robot_aux(std::map<int, float>& aux) = 0;
    
    
    // TBD can be auto-generated based on the TX PDO
    // NOTE setters
    virtual bool set_robot_pos_ref(const std::map<std::string, float>& pos_ref) = 0;
    virtual bool set_robot_pos_ref(const std::map<int, float>& pos_ref) = 0;
    
    virtual bool set_robot_vel_ref(const std::map<std::string, int16_t>& vel_ref) = 0;
    virtual bool set_robot_vel_ref(const std::map<int, int16_t>& vel_ref) = 0;
    
    virtual bool set_robot_tor_ref(const std::map<std::string, int16_t>& tor_ref) = 0;
    virtual bool set_robot_tor_ref(const std::map<int, int16_t>& tor_ref) = 0;
    
    virtual bool set_robot_gains(const std::map<std::string, std::vector<uint16_t> >& gains) = 0;
    virtual bool set_robot_gains(const std::map<int, std::vector<uint16_t> >& gains) = 0;
    
    virtual bool set_robot_fault_ack(const std::map<std::string, int16_t>& fault_ack) = 0;
    virtual bool set_robot_fault_ack(const std::map<int, int16_t>& fault_ack) = 0;
    
    virtual bool set_robot_ts(const std::map<int, uint16_t>& ts) = 0;    
    virtual bool set_robot_ts(const std::map<std::string, uint16_t>& ts) = 0;
    
    virtual bool set_robot_op_idx_aux(const std::map<int, uint16_t>& op_idx_aux) = 0;    
    virtual bool set_robot_op_idx_aux(const std::map<std::string, uint16_t>& op_idx_aux) = 0;
    
    virtual bool set_robot_aux(const std::map<std::string, float>& aux) = 0;
    virtual bool set_robot_aux(const std::map<int, float>& aux) = 0;


    virtual ~IXBotRobot() {
        printf("~IXBotRobot\n");
    };
};

#endif //__I_X_BOT_ROBOT_H__
