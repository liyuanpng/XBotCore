/*
 * Copyright (C) 2016 IIT-ADVR
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

#include <homing_example.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(HomingExample_factory, XBot::HomingExample, XBot::XBotControlPlugin);

namespace XBot {

HomingExample::HomingExample()
{

}

bool HomingExample::init_control_plugin(std::string path_to_config_file, RobotInterface::Ptr robot)
{
    _robot = robot;
    
    _robot->getRobotState("home_weird", _q_home);
    _robot->sense();
    _robot->getJointPosition(_q0);
    
//     _q_home *= -1;
    
    std::cout << "_q0 from SRDF : " << _q0 << std::endl;
    _time = 0;

    _robot->print();
    
    return true;
}

void HomingExample::control_loop(double time, double period)
{
    _robot->setPositionReference(_q0 + 0.5*(1-std::cos(0.5*(time - get_first_loop_time())))*(_q_home-_q0));
    _robot->move();
    
//     _robot->sense();
//     _robot->setReferenceFrom(_robot->model(), XBot::Sync::Position);
//     _robot->move();
    
//     _robot->sense();
//     _robot->print();
    
//      _robot->printTracking();    
    
    _time += 0.001;
}

bool HomingExample::close()
{
    return true;
}




    
    
}
