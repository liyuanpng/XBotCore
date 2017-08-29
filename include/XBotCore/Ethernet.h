
#ifndef __X_BOT_ETHERNET_H__
#define __X_BOT_ETHERNET_H__

#include <XBotCore-interfaces/XBotESC.h>
#include <RobotControlInterface.h>
#include <iostream>
namespace XBot
{
    class Ethernet;
}

/**
 * @brief XBotCore EtherCAT class.
 * 
 */
class XBot::Ethernet : public RobotControlInterface
                      
{
public:
    
    Ethernet(const char * config_yaml);
    virtual ~Ethernet();
   
    virtual void init() {
          std::cout<<"init ethernet"<<std::endl;
          
          
    };
      virtual int recv_from_slave(){std::cout<<"recvfrom ethernet"<<std::endl;};
      virtual int send_to_slave(){std::cout<<"sendto ethernet"<<std::endl;};
        
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
//     std::string get_thread_name(void);


protected:
    
    /**
     * @brief Robot control initialization function: called by the init_OP, before the ECAT thread loop: we are in OPERATIE state
     * 
     * @param  void
     * @return void
     */
//     virtual void control_init(void) = 0;

    /**
     * @brief Robot control loop
     * 
     * @param  void
     * @return 1 on success. 0 otherwise
     */
//     virtual int control_loop(void) = 0;
        


private:   
    
    /**
     * @brief The thread name
     * 
     */
//     std::string thread_name;
    
    /**
     * @brief SDO info XDDP Pipes
     * 
     */
//     std::map<int, std::shared_ptr<XDDP_pipe> > sdo_xddps;
    
    /**
     * @brief initialize the SDO XDDP pipes
     * 
     * @return void
     */
//     void init_sdo_xddp();
    
    /**
     * @brief write the sdo_info of the motor boards to the SDP XDDP pipes
     * 
     * @return void
     */
//     void write_sdo_info();
        
    /**
     * @brief Setter for the thread name
     * 
     * @param  std::string the thread name
     * @return void
     */
//     void set_thread_name(std::string);
        
    /**
     * @brief Setter for the thread period
     * 
     * @param  t the task period
     * @return void
     */
//     void set_thread_period(task_period_t t);
    
    /**
     * @brief Setter for the thread priority: RT thread
     * 
     * @param void
     * @return void
     */
//     void set_thread_priority();
    
    //     // NOTE IXBotHand getters/setters
    virtual double get_grasp_state(int hand_id){};
    virtual bool   grasp(int hand_id, double grasp_percentage){};
//     
//     // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) {};
    virtual bool get_ft_fault(int ft_id, double& fault) {};
    virtual bool get_ft_rtt(int ft_id, double& rtt) {};
//     
//     // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion){};
    virtual bool get_imu_fault(int imu_id, double& fault){};
    virtual bool get_imu_rtt(int imu_id, double& rtt){};

    
    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) {std::cout<<"getlinkpos ETHERNET"<<std::endl;};
    
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
//     
//     // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) {};
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) {};
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) {};
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) {};
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) {};
    
    virtual bool set_ts(int joint_id, const double& ts) {};
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) {};
    
    virtual bool set_aux(int joint_id, const double& aux) {};

};

#endif //__X_BOT_ECAT_H__
