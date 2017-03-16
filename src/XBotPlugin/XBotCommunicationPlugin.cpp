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
    _robot = robot;

    for( int id : _robot->getEnabledJointId() ) {
        _sub_map[id] = XBot::SubscriberRT<XBot::RobotState::pdo_tx>(std::string("rt_in_Motor_id_") + std::to_string(id));
    }

    return true;
}

void XBot::XBotCommunicationPlugin::control_loop(double time, double period)
{
    XBot::JointIdMap pos_ref_map, vel_ref_map, tor_ref_map;
    for( auto& p: _sub_map) {
        if( p.second.read(_pdo_tx) ) {
            pos_ref_map[p.first] = _pdo_tx.pos_ref;
            vel_ref_map[p.first] = _pdo_tx.vel_ref;
            tor_ref_map[p.first] = _pdo_tx.tor_ref;
        }
    }

    _robot->setPositionReference(pos_ref_map); 
    _robot->setVelocityReference(vel_ref_map); 
    _robot->setEffortReference(tor_ref_map); 
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

