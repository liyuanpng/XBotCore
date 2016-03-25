/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
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
 * @brief TBD
 * 
 */
class XBot::IXBotJoint
{

private:   

    // TBD can be auto-generated based on the PDO
    
    virtual bool get_link_pos(int joint_id, float& link_pos) = 0;
    
    virtual bool get_motor_pos(int joint_id, float& motor_pos) = 0;
    
    virtual bool get_link_vel(int joint_id, float& link_vel) = 0;
    
    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel) = 0;
    
    virtual bool get_torque(int joint_id, int16_t& torque) = 0;
    
    virtual bool get_max_temperature(int joint_id, uint16_t& max_temperature) = 0;
    
    virtual bool get_fault(int joint_id, uint16_t& fault) = 0;
    
    virtual bool get_rtt(int joint_id, uint16_t& rtt) = 0;
    
    virtual bool get_op_idx_ack(int joint_id, uint16_t& op_idx_ack) = 0;
    
    virtual bool get_aux(int joint_id, float& aux) = 0;
    

};

#endif //__I_X_BOT_JOINT_H__
