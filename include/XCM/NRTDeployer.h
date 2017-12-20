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

#ifndef __XBOT_NRT_DEPLOYER__
#define __XBOT_NRT_DEPLOYER__

#include <XCM/XBotThread.h>
#include <XCM/XBotPluginHandler.h>

#include <XBotCore-interfaces/XDomainCommunication.h>
#include <XCM/Loader.h>

namespace XBot
{

class NRTDeployer : public Thread_hook
{
public:

    NRTDeployer(std::string path_to_config);

    virtual void th_init(void*);
    virtual void th_loop(void*);

    virtual ~NRTDeployer();

protected:

private:
    
    double get_time();

    PluginHandler::Ptr _plugin_handler;
    RobotInterface::Ptr _robot;
    std::string _path_to_config;
    std::shared_ptr<Loader> _loader;
};

}

#endif
