/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_ECAT_H__
#define __X_BOT_ECAT_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/ecat/advr/esc.h>

/**
 * @brief TBD
 * 
 */
class XBotEcat : public Ec_Thread_Boards_base
{
public:
    
    XBotEcat(const char * config_yaml);
    virtual ~XBotEcat();
   
    /**
     * @brief initialization function called before the EtherCAT PRE-OPERATIONAL state : overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return void
     */
    virtual void init_preOP(void) final;
    
    /**
     * @brief initialization function called before the EtherCAT OPERATIONAL state , it calls the control_init function: overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return void
     */
    virtual void init_OP(void) final;
     
    /**
     * @brief Simply call the control loop function that will be implemented by the derived class: overridden from Ec_Thread_Boards_base
     * 
     * @param  void
     * @return 1 on plugin_handler_loop() success. 0 otherwise
     */
    virtual int user_loop(void) final;
        
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
    std::string get_thread_name(void);


protected:
    
    /**
     * @brief Robot control initialization function: called by the init_OP
     * 
     * @param  void
     * @return void
     */
    virtual void control_init(void) = 0;

    /**
     * @brief Robot control loop
     * 
     * @param  void
     * @return 1 on success. 0 otherwise
     */
    virtual int control_loop(void) = 0;
        


private:   
    
    /**
     * @brief The thread name
     * 
     */
    std::string thread_name;
        
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

#endif //__X_BOT_ECAT_H__
