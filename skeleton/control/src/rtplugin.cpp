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

#include <_MODULE_PREFIX_Plugin.h>

/* Specify that the class MiscPlugins::Homing is a Xbot RT plugin with name "Homing" */
REGISTER_XBOT_PLUGIN(_MODULE_PREFIX_Plugin, MiscPlugins::Homing)

namespace XBotPlugin {

bool Homing::init_control_plugin(std::string path_to_config_file,
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

void Homing::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

    /* Save the robot starting config to a variable */
    _robot->getMotorPosition(_q0);
}

void Homing::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */
}


void Homing::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

}

bool Homing::close()
{
    return true;
}



}