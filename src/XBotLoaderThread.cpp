/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
 *
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

#include <XBotCore/XBotLoaderThread.h>


XBot::XBotLoaderThread::XBotLoaderThread(std::shared_ptr<Loader> loaderptr) //const char* config_yaml, const char* param)   
{
  
    int period = 5000;
      
    set_thread_name("Loader");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    loader = loaderptr;
}

void XBot::XBotLoaderThread::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotLoaderThread::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotLoaderThread::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotLoaderThread::set_thread_priority()
{

    // set scheduler policy
#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotLoaderThread::th_init( void * ){
  
  loader->init_internal();
    
}

void XBot::XBotLoaderThread::th_loop( void * ){  
  
  loader->loop_internal();
   
}

XBot::XBotLoaderThread::~XBotLoaderThread() {
    
    printf("~XBotLoaderThread()\n");
}
