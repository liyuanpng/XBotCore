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

#ifndef __X_BOT_ECAT_H__
#define __X_BOT_ECAT_H__

#include <iit/advr/ec_boards_base.h>
#include <XBotCore-interfaces/XBotESC.h>

namespace XBot
{
    class XBotEcat;
}

/**
 * @brief XBotCore EtherCAT class.
 * 
 */
class XBot::XBotEcat : public Ec_Thread_Boards_base
                      
{
public:
    
    XBotEcat(const char * config_yaml);
    virtual ~XBotEcat();
   
    /**
     * @brief initialization function called before the EtherCAT OPERATIONAL state : overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return void
     */
    virtual void init_preOP(void) final;
    
    /**
     * @brief initialization function called just after the EtherCAT OPERATIONAL state , it calls the control_init function: overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return void
     */
//     virtual void init_OP(void) final;     
   
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
    std::string get_thread_name(void);

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

private:   
    
    /**
     * @brief The thread name
     * 
     */
    std::string thread_name;
    
    /**
     * @brief SDO info XDDP Pipes
     * 
     */
    std::map<int, std::shared_ptr<XDDP_pipe> > sdo_xddps;
    
    /**
     * @brief initialize the SDO XDDP pipes
     * 
     * @return void
     */
    void init_sdo_xddp();
    
    /**
     * @brief write the sdo_info of the motor boards to the SDP XDDP pipes
     * 
     * @return void
     */
    void write_sdo_info();
        
    /**
     * @brief Setter for the thread name
     * 
     * @param  std::string the thread name
     * @return void
     */
    void set_thread_name(std::string);
        
    /**
     * @brief Setter for the thread period
     * 
     * @param  t the task period
     * @return void
     */
    void set_thread_period(task_period_t t);
    
    /**
     * @brief Setter for the thread priority: RT thread
     * 
     * @param void
     * @return void
     */
    void set_thread_priority();
    
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

    

};

#endif //__X_BOT_ECAT_H__
