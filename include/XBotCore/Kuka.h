/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:  Giuseppe Rigano
 * email:   giuseppe.rigano@iit.it
 *
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


#ifndef __X_BOT_KUKA_H__
#define __X_BOT_KUKA_H__

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include "friudp.h"
#include "friremote.h"
#include <XBotCore/HALInterface.h>

namespace XBot
{
    class Kuka;
}

class XBot::Kuka :  public HALInterface {

private:
  
    //NOTE IXBotJoint getters
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
    
    friRemote friInst;
    FRI_QUALITY lastQuality;
    double timeCounter;
    float JntVals[LBR_MNJ];

public:
    Kuka(const char * config);
    virtual ~Kuka();

    void init_internal();
   
    virtual void init();
    virtual int recv_from_slave();
    virtual int send_to_slave();
    
    // NOTE IXBotHand getters/setters
    virtual double get_grasp_state(int hand_id);
    virtual bool   grasp(int hand_id, double grasp_percentage);
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) final;
    virtual bool get_ft_fault(int ft_id, double& fault) final;
    virtual bool get_ft_rtt(int ft_id, double& rtt) final;
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion);
    virtual bool get_imu_fault(int imu_id, double& fault);
    virtual bool get_imu_rtt(int imu_id, double& rtt);
};



#endif 
