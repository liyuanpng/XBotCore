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

#include <iit/advr/ec_boards_base.h>
#include <iit/ecat/advr/esc.h>

#include <XBotCore/XBotEcat.h>

#include <XBotCore/XBotCoreModel.hpp>


typedef std::map<int, std::string>  Rid2JointMap;
typedef std::map<std::string, int>  Joint2RidMap;


/**
 * @brief TBD
 * 
 */
class XBotCore : public XBotEcat
{
public:
    
    XBotCore(const char * config_yaml);
    virtual ~XBotCore();
    
    /**
     * @brief initialization function called before the control_loop
     * 
     * @param  void
     * @return void
     */
    virtual void control_init(void) final;
     
    /**
     * @brief Simply call the plugin handler loop function that will be implemented by the derived class: overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return 1 on plugin_handler_loop() success. 0 otherwise
     */
    virtual int control_loop(void) final;
    
    /**
     * @brief Plugin handler initialization: call the init() function of each plugin loaded
     * 
     * @param  void
     * @return true if all the plugin init() functions were successful, false otherwise
     */
    virtual bool plugin_handler_init(void) = 0;
    
    /**
     * @brief Plugin handler loop: call the run() function of each plugin loaded
     * 
     * @param  void
     * @return true if all the plugin run() functions were successful, false otherwise
     */
    virtual bool plugin_handler_loop(void) = 0;
    
    /**
     * @brief Getter for the robot model
     * 
     * @param  void
     * @return XBotCoreModel the model of the robot loaded in XBotCore
     */
    XBotCoreModel get_robot_model(void);
    
    /**
     * @brief Getter for the URDF path
     * 
     * @param  void
     * @return std::string the URDF path to load
     */
    std::string get_urdf_path(void);
    
    /**
     * @brief Getter for the SRDF path
     * 
     * @param  void
     * @return std::string the SRDF path to load
     */
    std::string get_srdf_path(void);
    


protected:
        
    /**
     * @brief map between joint robot id and joint name
     * 
     */
    Rid2JointMap rid2joint;
    
    /**
     * @brief map between joint name and joint robot id
     * 
     */
    Joint2RidMap joint2rid;
    
    std::vector<std::string> get_chain_names();
    
    bool get_chain_link_pos(std::string chain_name, std::map<std::string, float>& link_pos);

    bool get_chain_rtt(std::string chain_name, std::map<std::string, uint16_t>& rtt);


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
     * @brief map between the chain name and the id of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<int>> robot;
    
    
    
    std::string joint_map_config;
    YAML::Node joint_map_cfg;
    
    
    void parseJointMap(void);
    
    void generateRobot(void);
    
    
    virtual bool get_link_pos(int joint_id, float& link_pos);
    
    virtual bool get_motor_pos(int joint_id, float& motor_pos);
    
    virtual bool get_link_vel(int joint_id, float& link_vel);
    
    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel);
    
    virtual bool get_torque(int joint_id, int16_t& torque);
    
    virtual bool get_max_temperature(int joint_id, uint16_t& max_temperature);
    
    virtual bool get_fault(int joint_id, uint16_t& fault);
    
    virtual bool get_rtt(int joint_id, uint16_t& rtt);
    
    virtual bool get_op_idx_ack(int joint_id, uint16_t& op_idx_ack);
    
    virtual bool get_aux(int joint_id, float& aux);

    

};

#endif //__X_BOT_CORE_H__
