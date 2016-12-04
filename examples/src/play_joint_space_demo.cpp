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

#include <play_joint_space_demo.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(PlayJointSpaceDemo_factory, demo::PlayJointSpaceDemo, XBot::XBotControlPlugin);

namespace demo {
    
bool PlayJointSpaceDemo::init_control_plugin(std::string path_to_config_file, 
                                             XBot::SharedMemory::Ptr shared_memory, 
                                             XBot::RobotInterface::Ptr robot)
{
    _robot = robot;
    
    robot->getRobotState("home", _q_home);
    _generator = std::make_shared<JointTrajectoryGenerator>(robot, 20, _q_home);
    
    _robot->sense();
    _robot->getJointPosition(_q0);
    _homing_time = 5;
    return true;
}

void PlayJointSpaceDemo::control_loop(double time, double period)
{
    _robot->sense();
    
    if( (time - get_first_loop_time()) <= _homing_time ){
        _robot->setPositionReference(_q0 + 0.5*(1-std::cos(3.1415*(time - get_first_loop_time())/_homing_time))*(_q_home-_q0));
        _robot->move();
        return;
    }
    
    _generator->getQ(period, _q);
    _robot->setPositionReference(_q);
    
//     _robot->printTracking();
    _robot->move();
}

bool PlayJointSpaceDemo::close()
{
    return true;
}

    
}