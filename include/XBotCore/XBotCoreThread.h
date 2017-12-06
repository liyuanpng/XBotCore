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


#ifndef __X_BOT_CORE_THREAD_H__
#define __X_BOT_CORE_THREAD_H__

#include <XCM/XBotUtils.h>
#include <XCM/XBotThread.h>
#include <XBotCore/XBotCore.h>
#include <memory>


namespace XBot
{
    class XBotCoreThread;  
}

/**
 * @brief XBotCore: RT EtherCAT thread and RT (shared-memory) XBotCore interfaces implementation.
 * 
 */
class XBot::XBotCoreThread : public XBot::Thread_hook
                        
{
public:
    
    XBotCoreThread() = default;
    
    XBotCoreThread(std::string config_yaml, 
                   XBot::SharedMemory::Ptr shared_memory,  
                   Options options, 
                   HALInterface::Ptr hal = nullptr,
                   std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider = nullptr
                   
                  );
    
    virtual ~XBotCoreThread();    
    
    virtual void th_init ( void * );
    virtual void th_loop ( void * );
        
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
    std::string get_thread_name(void);

private:
    
    std::shared_ptr<ControllerInterface> controller;
    
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

};

#endif //__X_BOT_CORE_H__
