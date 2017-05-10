/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: 
 * email: 
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

#include <_MODULE_PREFIX__rt_plugin.h>

/* Specify that the class XBotPlugin::_MODULE_PREFIX__rt_plugin is a XBot RT plugin with name "_MODULE_PREFIX__rt_plugin" */
REGISTER_XBOT_PLUGIN(_MODULE_PREFIX__rt_plugin, XBotPlugin::_MODULE_PREFIX__rt_plugin)

namespace XBotPlugin {

bool _MODULE_PREFIX__rt_plugin::init_control_plugin(std::string path_to_config_file,
                                 XBot::SharedMemory::Ptr shared_memory,
                                 XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;

    return true;


}

void _MODULE_PREFIX__rt_plugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

    /* Save the robot starting config to a variable */
    _robot->getMotorPosition(_q0);
}

void _MODULE_PREFIX__rt_plugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */
}


void _MODULE_PREFIX__rt_plugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

}

bool _MODULE_PREFIX__rt_plugin::close()
{
    return true;
}



}