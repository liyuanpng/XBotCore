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
#include <XBotCore-interfaces/XBotRosUtils.h>
#include <geometry_msgs/Point.h>

REGISTER_XBOT_PLUGIN_(XBot::HomingExample)

namespace XBot {

HomingExample::HomingExample()
{

}

bool HomingExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();

    _robot->getRobotState("home", _q_home);
    _robot->sense();
    _robot->getJointPosition(_q0);
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    _k = _k0;
    _d = _d0;
    _q = _q0;
    _qref = _q0;

    _time = 0;
    _homing_time = 4;


    _l_hand_pos = _l_hand_ref = 0.0;
    _close_hand = true;
    
    _pub_rt = handle->getRosHandle()->advertiseTopic<geometry_msgs::Point>("test_topic", 1);
    
    
    

//     _robot->initLog("/tmp/homing_example_log", 100000);

    return true;


}

void HomingExample::on_start(double time)
{
    _first_loop_time = time;
    _robot->sense();
    _robot->getJointPosition(_q0);
    
   
    
// NOTE if you want to grasp use this piece of code    
//     _robot->setPositionReference(_q0);
//     
//       
//     //r hand open
//     int r_hand_id =_robot->getHand()["r_handj"]->getHandId();
//     XBot::Hand::Ptr r_hand =_robot->getHand(r_hand_id);
//     r_hand->grasp(0.5);
//     
//     //l hand closing
//     int l_hand_id =_robot->getHand()["l_handj"]->getHandId();
//     XBot::Hand::Ptr l_hand =_robot->getHand(l_hand_id);
//     l_hand->grasp(0.0);
//    
//     _robot->move();
    
    
}

void HomingExample::on_stop(double time)
{
}


void HomingExample::control_loop(double time, double period)
{

    
    geometry_msgs::Point msg;
    msg.x = time;
    msg.y = period;
    msg.z = 1;
    
    _pub_rt->pushToQueue(msg);

        if(current_command.str() == "MY_COMMAND_1"){
            /* Handle command */
            std::cout<<"MY_COMMAND_1qq"<<std::endl;
        }

        if(current_command.str() == "MY_COMMAND_2"){
            /* Handle command */
             std::cout<<"MY_COMMAND_2"<<std::endl;
        }

  
//     _robot->sense();
//     _robot->log(time);

   // Go to homing
    if( (time - _first_loop_time) <= _homing_time ){
        _q = _q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0);
        _robot->setPositionReference(_q);
        _robot->move();
        return;

    }

//     _robot->move();

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
    Logger::info() << "HomingExample::close()" << Logger::endl();
    return true;
}






}
