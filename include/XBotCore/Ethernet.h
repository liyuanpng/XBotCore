/*
   Boards_ctrl_basic.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/


#ifndef __X_BOT_ETHERNET_H__
#define __X_BOT_ETHERNET_H__


#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>
#include <XBotCore/HALInterface.h>

/**
 * @class Boards_ctrl_basic
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_basic class
 */

class Ethernet : /*public Thread_hook,*/ 
                          public HALInterface,
                          public Boards_ctrl_ext  {

private:
    uint64_t g_tStart;
    uint8_t trj_flag;

    group_ref_t pos_group;
    group_ref_comp_t pos_vel_group;

    double control_old, vel_old;

    Write_XDDP_pipe * xddp_test;
    
    
    //NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) {};
    
    virtual bool get_motor_pos(int joint_id, double& motor_pos) {};
    
    virtual bool get_link_vel(int joint_id, double& link_vel) {};
    
    virtual bool get_motor_vel(int joint_id, double& motor_vel) {};
    
    virtual bool get_torque(int joint_id, double& torque) {};
    
    virtual bool get_temperature(int joint_id, double& temperature) {};
    
    virtual bool get_fault(int joint_id, double& fault) {};
    
    virtual bool get_rtt(int joint_id, double& rtt) {};
    
    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) {};
    
    virtual bool get_aux(int joint_id, double& aux) {};
    
    virtual bool get_gains(int joint_id, std::vector< double >& gain_vector) {};
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) {};
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) {};
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) {};
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) {};
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) {};
    
    virtual bool set_ts(int joint_id, const double& ts) {};
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) {};
    
    virtual bool set_aux(int joint_id, const double& aux) {};

public:
    Ethernet(const char * config);
    virtual ~Ethernet();

//     virtual void th_init(void *);
//     virtual void th_loop(void *);

//     int user_input(uint8_t &cmd);
//     int user_loop(void);
    
    void init_internal();
    
    
    virtual void init();
    virtual int recv_from_slave();
    virtual int send_to_slave();
    
     // NOTE IXBotHand getters/setters
    virtual double get_grasp_state(int hand_id){};
    virtual bool   grasp(int hand_id, double grasp_percentage){};
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) {};
    virtual bool get_ft_fault(int ft_id, double& fault) {};
    virtual bool get_ft_rtt(int ft_id, double& rtt) {};
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion){};
    virtual bool get_imu_fault(int imu_id, double& fault){};
    virtual bool get_imu_rtt(int imu_id, double& rtt){};


};



#endif 
