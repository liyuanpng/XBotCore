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

#include <XBotPlugin/XBotLoggingPlugin.h>
#include <XCM/XBotUtils.h>

REGISTER_XBOT_PLUGIN_(XBot::XBotLoggingPlugin)

XBot::XBotLoggingPlugin::XBotLoggingPlugin()
{

}

bool XBot::XBotLoggingPlugin::init_control_plugin( XBot::Handle::Ptr handle )
{
    // get the robot
    _robot = handle->getRobotInterface();

    // initialize logger
    _logger = XBot::MatLogger::getLogger("/tmp/XBotCore_log");
    _robot->initLog(_logger, 100000);

    // add the faults
    _faults.resize(_robot->getJointNum(), 0.0);
    _logger->createVectorVariable("fault", _faults.size(), 1, 100000);

    // add the aux
    _aux.resize(_robot->getJointNum(), 0.0);
    _logger->createVectorVariable("current", _aux.size(), 1, 100000);

    return true;
}

void XBot::XBotLoggingPlugin::on_start(double time)
{
    DPRINTF("Start Logging ...\n");
}

void XBot::XBotLoggingPlugin::on_stop(double time)
{
   DPRINTF("Stop Logging ...\n");
}


void XBot::XBotLoggingPlugin::control_loop(double time, double period)
{
    // log all robot state
    _robot->log(_logger, time);

    // log fault
    int i = 0;
    for(int id : _robot->getEnabledJointId()) {
        if(get_xbotcore_joint()) {
            get_xbotcore_joint()->get_fault(id, _faults[i]);
        }
            i++;
    }
    _logger->add("fault", _faults);

    // log aux
    i = 0;
    for(int id : _robot->getEnabledJointId()) {
        if(get_xbotcore_joint()) {
            get_xbotcore_joint()->get_aux(id, _aux[i]);
        }
            i++;
    }
    _logger->add("current", _aux);


}

bool XBot::XBotLoggingPlugin::close(void)
{
    _logger->flush();
    return true;
}

XBot::XBotLoggingPlugin::~XBotLoggingPlugin()
{
    Logger::info() << "~XBotLoggingPlugin()" << Logger::endl();
}

