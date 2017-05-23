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

    // create the CommunicationHandler thread
//     _ch = std::make_shared<XBot::CommunicationHandler>(path_to_config_file);
//     _ch->create(false, 3);

    return true;
}

void XBot::XBotCommunicationPlugin::on_start(double time)
{
    _start_time = time;
    _robot->getJointPosition(_q0);
}

void XBot::XBotCommunicationPlugin::on_stop(double time)
{

}


void XBot::XBotCommunicationPlugin::control_loop(double time, double period)
{

    for( auto& p: _sub_map) {
        if( p.second.read(_pdo_tx) ) {
            _pos_ref_map[p.first] = _pdo_tx.pos_ref;
            _vel_ref_map[p.first] = _pdo_tx.vel_ref;
            _tor_ref_map[p.first] = _pdo_tx.tor_ref;
        }
    }

    _robot->setPositionReference(_pos_ref_map);
    _robot->setVelocityReference(_vel_ref_map);
    _robot->setEffortReference(_tor_ref_map);

    double alpha = (time - _start_time) / 5.0;

    if( alpha < 1 ){
        _robot->getPositionReference(_qref);
        _qref = alpha*_qref + (1-alpha)*_q0;
        _robot->setPositionReference(_qref);
    }

    _robot->move();

}

bool XBot::XBotCommunicationPlugin::close(void)
{
    // stop communication handler thread
//     _ch->stop();
//     _ch->join();

    return true;
}

XBot::XBotCommunicationPlugin::~XBotCommunicationPlugin()
{
    printf("~XBotCommunicationPlugin()\n");
}

