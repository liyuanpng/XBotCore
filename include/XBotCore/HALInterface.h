#ifndef __X_BOT_ROBOTCONTROLINTERFACE_H__
#define __X_BOT_ROBOTCONTROLINTERFACE_H__

#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotFT.h>
#include <XBotCore-interfaces/IXBotIMU.h>
#include <XBotCore-interfaces/IXBotHand.h>

class HALInterface :  public XBot::IXBotJoint,
                      public XBot::IXBotFT,
                      public XBot::IXBotIMU,
                      public XBot::IXBotHand {
public:
    
    typedef std::shared_ptr<HALInterface> Ptr;
  
    virtual void init() = 0;
    virtual int recv_from_slave() = 0;
    virtual int send_to_slave() = 0;   
   
    // NOTE IXBotHand getters/setters
    virtual double get_grasp_state(int hand_id) = 0;
    virtual bool   grasp(int hand_id, double grasp_percentage) = 0;
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6)  = 0;
    virtual bool get_ft_fault(int ft_id, double& fault)  = 0;
    virtual bool get_ft_rtt(int ft_id, double& rtt) = 0;
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion) = 0;
    virtual bool get_imu_fault(int imu_id, double& fault) = 0;
    virtual bool get_imu_rtt(int imu_id, double& rtt) = 0;

    
    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) = 0;
    
    virtual bool get_motor_pos(int joint_id, double& motor_pos) = 0;
    
    virtual bool get_link_vel(int joint_id, double& link_vel) = 0;
    
    virtual bool get_motor_vel(int joint_id, double& motor_vel) = 0;
    
    virtual bool get_torque(int joint_id, double& torque) = 0;
    
    virtual bool get_temperature(int joint_id, double& temperature) = 0;
    
    virtual bool get_fault(int joint_id, double& fault) = 0;
    
    virtual bool get_rtt(int joint_id, double& rtt) = 0;
    
    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) = 0;
    
    virtual bool get_aux(int joint_id, double& aux) = 0;
    
    virtual bool get_gains(int joint_id, std::vector< double >& gain_vector) = 0;
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) = 0;
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) = 0;
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) = 0;
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) = 0;
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) = 0;
    
    virtual bool set_ts(int joint_id, const double& ts) = 0;
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) = 0;
    
    virtual bool set_aux(int joint_id, const double& aux) = 0;
};

#endif
