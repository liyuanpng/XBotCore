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

#ifndef __XCM_MODULE_HANDLER_H__
#define __XCM_MODULE_HANDLER_H__

#include <XCM/XBotControlModule.h>
#include <XCM/TimeProvider.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <XCM/XBotCommunicationInterface.h>

namespace XBot {

class ModuleHandler {

public:

    ModuleHandler(std::string path_to_config_file,
                  RobotInterface::Ptr robot,
                  XBotControlModule::Ptr control_module,
                  TimeProvider::Ptr time_provider
                  );

    bool init();

    void run();

    void close();

    ~ModuleHandler();


protected:


private:

    std::string _path_to_config_file;
    RobotInterface::Ptr _robot;
    TimeProvider::Ptr _time_provider;
    XBotControlModule::Ptr _module;

    std::vector<CommunicationInterface::Ptr> _communication_interfaces;

    double _time, _last_time, _period;
    bool _first_loop;

    std::string _state;

    bool _close_was_called;


};

}
#endif