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

bool HomingExample::init_control_plugin(std::string path_to_config_file,
                                        XBot::SharedMemory::Ptr shared_memory,
                                        RobotInterface::Ptr robot)
{
    _robot = robot;

    _robot->getRobotState("home", _q_home);
    _robot->sense();
    _robot->getJointPosition(_q0);
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    _k = _k0;
    _d = _d0;
    _q = _q0;

//     if( !_robot->checkJointLimits(_q_home) ) throw;

//     _q_home *= -1;

    std::cout << "_q_home from SRDF : " << _q_home << std::endl;
    _time = 0;
    _homing_time = 4;

    _robot->print();

    _l_hand_pos = _l_hand_ref = 0.0;
    _close_hand = true;

//     _robot->initLog("/tmp/homing_example_log", 100000);

    return true;


}

void HomingExample::on_start(double time)
{
    _first_loop_time = time;
    _robot->sense();
    _robot->getJointPosition(_q0);
    std::cout << name << " STARTED!!!" << std::endl;
}

void HomingExample::on_stop(double time)
{
    std::cout << name << " STOPPED!!!" << std::endl;
}


void HomingExample::control_loop(double time, double period)
{



    _robot->sense();
//     _robot->log(time);

   // Go to homing
    if( (time - _first_loop_time) <= _homing_time ){
        _q = _q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0);
        _robot->setPositionReference(_q);
        _robot->move();
        return;

    }
    
    _k = _k0 / 20;
    _d = _d0/20;
    
    _robot->setStiffness(_k);
    _robot->setDamping(_d);
    _robot->move();

//     if(_close_hand) {
//         _robot->sense();
//
//         _l_hand_pos =_robot->chain("left_hand").getJointPosition(0);
//         printf("reading %f\n", _l_hand_pos);
//
//         _l_hand_ref = 13;
//         _robot->chain("left_hand").setPositionReference(0, _l_hand_ref);
//         printf("ref %f\n", _l_hand_ref);
//
//         _robot->move();
//         _close_hand = false;
//     }

    // sense to get wrench
//     _robot->sense();
//     _l_arm_ft->getWrench(_l_arm_wrench);

//     _robot->sense();
//     _robot->setReferenceFrom(_robot->model(), XBot::Sync::Position);
//     _robot->move();

//     _robot->sense();
//     _robot->print();

//      _robot->printTracking();

//     _time += 0.001;
}

bool HomingExample::close()
{
//     _robot->flushLog();
    return true;
}






}
