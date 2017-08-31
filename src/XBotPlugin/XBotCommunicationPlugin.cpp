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

#include <XBotPlugin/XBotCommunicationPlugin.h>

REGISTER_XBOT_PLUGIN(XBotCommunicationPlugin, XBot::XBotCommunicationPlugin)

XBot::XBotCommunicationPlugin::XBotCommunicationPlugin()
{

}

bool XBot::XBotCommunicationPlugin::init_control_plugin(std::string path_to_config_file,
                                                        XBot::SharedMemory::Ptr shared_memory,
                                                        RobotInterface::Ptr robot)
{
    // get the robot
    _robot = robot;

    // create a SubscriberRT for each enabled joint in the robot
    for( int id : _robot->getEnabledJointId() ) {
        _sub_map[id] = XBot::SubscriberRT<XBot::RobotState::pdo_tx>(std::string("rt_in_Motor_id_") + std::to_string(id));
    }
    // and for hands
    for( const auto& h : _robot->getHand() ) {
        int id = h.second->getHandId();
        _sub_map[id] = XBot::SubscriberRT<XBot::RobotState::pdo_tx>(std::string("rt_in_Motor_id_") + std::to_string(id));
    }

    // initialize filter
    _filter_q = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*0.2, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    _filter_k = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*0.2, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    _filter_d = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*0.2, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    
    // NOTE filter ON by default
    _filter_enabled = true;
    _robot->getMotorPosition(_qref);
    _robot->getStiffness(_kref);
    _robot->getDamping(_dref);

    _filter_q.reset(_qref);
    _filter_k.reset(_kref);
    _filter_d.reset(_dref);
    
    DPRINTF("Filter ON by default\n");

    for (auto& p: _robot->getHand())
    {
        XBot::Hand::Ptr hand = p.second;
        _hand_map[hand->getHandId()] =  hand;
    }
    
    

    return true;
}

void XBot::XBotCommunicationPlugin::on_start(double time)
{
    _start_time = time;
    _robot->getJointPosition(_q0);
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);

    _filter_q.reset(_q0);
    _filter_k.reset(_k0);
    _filter_d.reset(_d0);

    _robot->getJointPosition(_pos_ref_map);
    _robot->getStiffness(_k_ref_map);
    _robot->getDamping(_d_ref_map);
}

void XBot::XBotCommunicationPlugin::on_stop(double time)
{

}


void XBot::XBotCommunicationPlugin::control_loop(double time, double period)
{

    if(command.read(current_command)){
        if(current_command.str() == "filter ON"){
            _filter_enabled = true;
            _robot->getJointPosition(_qref);
            _robot->getStiffness(_kref);
            _robot->getDamping(_dref);

            _filter_q.reset(_qref);
            _filter_k.reset(_kref);
            _filter_d.reset(_dref);
        }
        if(current_command.str() == "filter OFF"){
            _filter_enabled = false;
        }
    }
    
   
     
    for( auto& p: _sub_map) {
        if( p.second.read(_pdo_tx) ) {
          
            if( _hand_map[p.first] != nullptr){                
                // HACK scaling back based on 9.0 max range
                _hand_map[p.first]->grasp(_pdo_tx.pos_ref / 9.0);
            }          
            
            _pos_ref_map[p.first] = _pdo_tx.pos_ref;
            _vel_ref_map[p.first] = _pdo_tx.vel_ref;
            _tor_ref_map[p.first] = _pdo_tx.tor_ref;
            _k_ref_map[p.first] = _pdo_tx.gain_0;
            _d_ref_map[p.first] = _pdo_tx.gain_1;
        }
    }

    _robot->setPositionReference(_pos_ref_map);

    _robot->setVelocityReference(_vel_ref_map);
    _robot->setEffortReference(_tor_ref_map);
    _robot->setStiffness(_k_ref_map);
    _robot->setDamping(_d_ref_map);

    double alpha = (time - _start_time) / 5.0;

    if( alpha < 1 ){
        _robot->getPositionReference(_qref);
        _qref = alpha*_qref + (1-alpha)*_q0;
        _robot->setPositionReference(_qref);

        _robot->getStiffness(_kref);
        _kref = alpha*_kref + (1-alpha)*_k0;
        _robot->setStiffness(_kref);

        _robot->getDamping(_dref);
        _dref = alpha*_dref + (1-alpha)*_d0;
        _robot->setDamping(_dref);
    }

    if(_filter_enabled){
        _robot->getPositionReference(_qref);
        _robot->setPositionReference(_filter_q.process(_qref));

        _robot->getStiffness(_kref);
        _robot->setStiffness(_filter_k.process(_kref));

        _robot->getDamping(_dref);
        _robot->setDamping(_filter_d.process(_dref));
    }

    _robot->move();

}

bool XBot::XBotCommunicationPlugin::close(void)
{

    return true;
}

XBot::XBotCommunicationPlugin::~XBotCommunicationPlugin()
{
    printf("~XBotCommunicationPlugin()\n");
}


