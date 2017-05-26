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
     * @brief Robot control initialization function: called by the init_OP, before the ECAT thread loop: we are in OPERATIE state
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
    

};

#endif //__X_BOT_ECAT_H__
