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

#include <XBotCore/XBotCoreThread.h>


XBot::XBotCoreThread::XBotCoreThread(const char* config_yaml, const char* param)   
{
   // _path_to_config = config_yaml;
  //TODO fix thread stuff
    // set thread name
    //const YAML::Node& board_ctrl = root_cfg["x_bot_ecat"]; // TBD check that the Node is defined
    //set_thread_name(board_ctrl["name"].as<std::string>()); // TBD check that name is defined
    int period = 1;
    if (param != nullptr){      
       if(strcmp(param,"dummy") == 0){
         period = 1000;
      }      
    }
      
    set_thread_name("XBOT");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    controller = std::shared_ptr<ControllerInterface>(new XBot::XBotCore(config_yaml, param));
}

void XBot::XBotCoreThread::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotCoreThread::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotCoreThread::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotCoreThread::set_thread_priority()
{

    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotCoreThread::th_init( void * ){
  
    controller->control_init();
}

void XBot::XBotCoreThread::th_loop( void * ){  
  
    controller->control_loop();  
}

XBot::XBotCoreThread::~XBotCoreThread() {
    
   Logger::info() << "~XBotCoreThread()" << Logger::endl();
}
