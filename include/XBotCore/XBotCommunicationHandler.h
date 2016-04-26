/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_CORE_H__
#define __X_BOT_CORE_H__

#include <iit/ecat/advr/esc.h>
#include <iit/advr/thread_util.h>

#include <mutex>

#include <XBotCore/IXBotJoint.h>
#include <XBotCore/IXBotChain.h>
#include <XBotCore/IXBotRobot.h>

#include <XBotCore/XBotCoreModel.hpp>

namespace XBot
{
    class XBotCommunicationHandler;
}


/**
 * @brief TBD
 * 
 */
class XBot::XBotCommunicationHandler : public Thread_hook,
                                       public XBot::IXBotJoint/*,
                                       public   XBot::IXBotChain,
                                       public   XBot::IXBotRobot*/
                        
{
public:
    
    XBotCommunicationHandler(std::string config_file);
    virtual ~XBotCommunicationHandler();
    
    std::map<std::string,std::vector<int> > get_robot_map();
    
    virtual void th_init(void* );
    
    virtual void th_loop(void* );
    
//     // NOTE IXBotRobot getters
//     virtual bool get_robot_link_pos(std::map<std::string, float>& link_pos) final;
//     virtual bool get_robot_link_pos(std::map<int, float>& link_pos) final;
//     
//     virtual bool get_robot_motor_pos(std::map<std::string, float>& motor_pos) final;
//     virtual bool get_robot_motor_pos(std::map<int, float>& motor_pos) final;
//     
//     virtual bool get_robot_link_vel(std::map<std::string, float>& link_vel) final;
//     virtual bool get_robot_link_vel(std::map<int, float>& link_vel) final;
//     
//     virtual bool get_robot_motor_vel(std::map<std::string, int16_t>& motor_vel) final;
//     virtual bool get_robot_motor_vel(std::map<int, int16_t>& motor_vel) final;
//     
//     virtual bool get_robot_torque(std::map<std::string, int16_t>& torque) final;
//     virtual bool get_robot_torque(std::map<int, int16_t>& torque) final;
//     
//     virtual bool get_robot_max_temperature(std::map<int, uint16_t>& max_temperature) final;    
//     virtual bool get_robot_max_temperature(std::map<std::string, uint16_t>& max_temperature) final;
//     
//     virtual bool get_robot_fault(std::map<int, uint16_t>& fault) final;    
//     virtual bool get_robot_fault(std::map<std::string, uint16_t>& fault) final;
// 
//     virtual bool get_robot_rtt(std::map<int, uint16_t>& rtt) final;    
//     virtual bool get_robot_rtt(std::map<std::string, uint16_t>& rtt) final;
//     
//     virtual bool get_robot_op_idx_ack(std::map<int, uint16_t>& op_idx_ack) final;    
//     virtual bool get_robot_op_idx_ack(std::map<std::string, uint16_t>& op_idx_ack) final;
//     
//     virtual bool get_robot_aux(std::map<std::string, float>& aux) final;
//     virtual bool get_robot_aux(std::map<int, float>& aux) final;
// 
//     // NOTE IXBotRobot setters
//     virtual bool set_robot_pos_ref(const std::map<std::string, float>& pos_ref) final;
//     virtual bool set_robot_pos_ref(const std::map<int, float>& pos_ref) final;
//     
//     virtual bool set_robot_vel_ref(const std::map<std::string, int16_t>& vel_ref) final;
//     virtual bool set_robot_vel_ref(const std::map<int, int16_t>& vel_ref) final;
//     
//     virtual bool set_robot_tor_ref(const std::map<std::string, int16_t>& tor_ref) final;
//     virtual bool set_robot_tor_ref(const std::map<int, int16_t>& tor_ref) final;
//     
//     virtual bool set_robot_gains(const std::map<std::string, std::vector<uint16_t> >& gains) final;
//     virtual bool set_robot_gains(const std::map<int, std::vector<uint16_t> >& gains) final;
//     
//     virtual bool set_robot_fault_ack(const std::map<std::string, int16_t>& fault_ack) final;
//     virtual bool set_robot_fault_ack(const std::map<int, int16_t>& fault_ack) final;
//     
//     virtual bool set_robot_ts(const std::map<int, uint16_t>& ts) final;    
//     virtual bool set_robot_ts(const std::map<std::string, uint16_t>& ts) final;
//     
//     virtual bool set_robot_op_idx_aux(const std::map<int, uint16_t>& op_idx_aux) final;    
//     virtual bool set_robot_op_idx_aux(const std::map<std::string, uint16_t>& op_idx_aux) final;
//     
//     virtual bool set_robot_aux(const std::map<std::string, float>& aux) final;
//     virtual bool set_robot_aux(const std::map<int, float>& aux) final;
// 
//     // NOTE IXBotChain getters
//     virtual bool get_chain_link_pos(std::string chain_name, std::map<std::string, float>& link_pos) final;
//     virtual bool get_chain_link_pos(std::string chain_name, std::map<int, float>& link_pos) final;
//     
//     virtual bool get_chain_motor_pos(std::string chain_name, std::map<std::string, float>& motor_pos) final;
//     virtual bool get_chain_motor_pos(std::string chain_name, std::map<int, float>& motor_pos) final;
//     
//     virtual bool get_chain_link_vel(std::string chain_name, std::map<std::string, float>& link_vel) final;
//     virtual bool get_chain_link_vel(std::string chain_name, std::map<int, float>& link_vel) final;
//     
//     virtual bool get_chain_motor_vel(std::string chain_name, std::map<std::string, int16_t>& motor_vel) final;
//     virtual bool get_chain_motor_vel(std::string chain_name, std::map<int, int16_t>& motor_vel) final;
//     
//     virtual bool get_chain_torque(std::string chain_name, std::map<std::string, int16_t>& torque) final;
//     virtual bool get_chain_torque(std::string chain_name, std::map<int, int16_t>& torque) final;
//     
//     virtual bool get_chain_max_temperature(std::string chain_name, std::map<int, uint16_t>& max_temperature) final;    
//     virtual bool get_chain_max_temperature(std::string chain_name, std::map<std::string, uint16_t>& max_temperature) final;
//     
//     virtual bool get_chain_fault(std::string chain_name, std::map<int, uint16_t>& fault) final;    
//     virtual bool get_chain_fault(std::string chain_name, std::map<std::string, uint16_t>& fault) final;
// 
//     virtual bool get_chain_rtt(std::string chain_name, std::map<int, uint16_t>& rtt) final;    
//     virtual bool get_chain_rtt(std::string chain_name, std::map<std::string, uint16_t>& rtt) final;
//     
//     virtual bool get_chain_op_idx_ack(std::string chain_name, std::map<int, uint16_t>& op_idx_ack) final;    
//     virtual bool get_chain_op_idx_ack(std::string chain_name, std::map<std::string, uint16_t>& op_idx_ack) final;
//     
//     virtual bool get_chain_aux(std::string chain_name, std::map<std::string, float>& aux) final;
//     virtual bool get_chain_aux(std::string chain_name, std::map<int, float>& aux) final;
//     
//     // NOTE IXBotChain setters
//     virtual bool set_chain_pos_ref(std::string chain_name, const std::map<std::string, float>& pos_ref) final;
//     virtual bool set_chain_pos_ref(std::string chain_name, const std::map<int, float>& pos_ref) final;
//     
//     virtual bool set_chain_vel_ref(std::string chain_name, const std::map<std::string, int16_t>& vel_ref) final;
//     virtual bool set_chain_vel_ref(std::string chain_name, const std::map<int, int16_t>& vel_ref) final;
//     
//     virtual bool set_chain_tor_ref(std::string chain_name, const std::map<std::string, int16_t>& tor_ref) final;
//     virtual bool set_chain_tor_ref(std::string chain_name, const std::map<int, int16_t>& tor_ref) final;
//     
//     virtual bool set_chain_gains(std::string chain_name, const std::map<std::string, std::vector<uint16_t> >& gains) final;
//     virtual bool set_chain_gains(std::string chain_name, const std::map<int, std::vector<uint16_t> >& gains) final;
//     
//     virtual bool set_chain_fault_ack(std::string chain_name, const std::map<std::string, int16_t>& fault_ack) final;
//     virtual bool set_chain_fault_ack(std::string chain_name, const std::map<int, int16_t>& fault_ack) final;
//     
//     virtual bool set_chain_ts(std::string chain_name, const std::map<int, uint16_t>& ts) final;    
//     virtual bool set_chain_ts(std::string chain_name, const std::map<std::string, uint16_t>& ts) final;
//     
//     virtual bool set_chain_op_idx_aux(std::string chain_name, const std::map<int, uint16_t>& op_idx_aux) final;    
//     virtual bool set_chain_op_idx_aux(std::string chain_name, const std::map<std::string, uint16_t>& op_idx_aux) final;
//     
//     virtual bool set_chain_aux(std::string chain_name, const std::map<std::string, float>& aux) final;
//     virtual bool set_chain_aux(std::string chain_name, const std::map<int, float>& aux) final;
    
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
     * @brief fd reading from pipes: we read the robot from XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_read;
    
    /**
     * @brief fd writing to pipes: we writhe to the robot with XBotCore XDDP pipe
     * 
     */
    std::map<int,int> fd_write;
    
    int n_bytes;
    
    /**
     * @brief mutex
     * 
     */
    std::map<int,std::shared_ptr<std::mutex>> mutex;
    
    
    std::map<int, std::shared_ptr<iit::ecat::advr::McEscPdoTypes>> pdo;
    


    

};

#endif //__X_BOT_CORE_H__
