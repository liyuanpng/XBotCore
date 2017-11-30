/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
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

#include <XCM/NRTDeployer.h>

#include <XCM/XBotUtils.h>

#include <boost/bind.hpp>

XBot::NRTDeployer::NRTDeployer(std::string path_to_config) :
    _path_to_config(path_to_config)
{
    // set thread name
    name = "NRTDeployer";
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,5000};
    period.task_time = t.task_time;
    period.period = t.period;
    // set scheduler policy
    schedpolicy = SCHED_OTHER;
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

double XBot::NRTDeployer::get_time()
{
    return XBot::get_time_ns() / 1e9;
}


void XBot::NRTDeployer::th_init(void*)
{

    // XBot robot
    _robot = RobotInterface::getRobot(_path_to_config);
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::NRTDeployer::get_time, this);
    
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    
    // shared memory
    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();
    
    Options opt;
    opt.xbotcore_pluginhandler_plugin_set_name = "NRTPlugins";
    
    _plugin_handler = std::make_shared<PluginHandler>(_robot, time_provider, shared_memory, opt);

    
    
    // loading and init_plugins
    _plugin_handler->load_plugins();
    _plugin_handler->init_plugins();
    _loader = std::make_shared<Loader>(_plugin_handler);
    _loader->init_internal();


}

void XBot::NRTDeployer::th_loop(void*)
{
    _plugin_handler->run();
    _loader->loop_internal();
}

XBot::NRTDeployer::~NRTDeployer()
{
    _plugin_handler->close();
}


