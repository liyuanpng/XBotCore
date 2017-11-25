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
    
    _pub_rt = handle->getRosHandle()->advertise<geometry_msgs::Point>("test_topic", 10);
    _pub_rt_1 = handle->getRosHandle()->advertise<sensor_msgs::JointState>("test_topic_js", 10);
    _sub_rt = handle->getRosHandle()->subscribe<std_msgs::Float64>("test_homing_time", 1, &HomingExample::callback, this);
    _srv_rt = handle->getRosHandle()->advertiseService("test_service", &HomingExample::srv_callback, this);

    _js_msg.name = {"IO", "SONO", "LEGGENDA"};
    _js_msg.position.resize(50);
    _js_msg.effort.resize(50);
    
    _nrt_ref_sh = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("pos_ref_map_so");
    

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
    msg.x = period;
    
    _pub_rt->pushToQueue(msg);
    
    _nrt_ref_sh.get(_pos_ref_map);
    
    // go to homing
    if( (time - _first_loop_time) <= _homing_time ) {
        _q = _q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0);
        _robot->setPositionReference(_q);
        _robot->move();
        return;
    }

    // after we arrive in the homing position read head reference from the NRT
    
    

//     _robot->chain("head").setPositionReference(_pos_ref_map); 
    _robot->move();
}

bool HomingExample::close()
{
    Logger::info() << "HomingExample::close()" << Logger::endl();
    return true;
}






}
