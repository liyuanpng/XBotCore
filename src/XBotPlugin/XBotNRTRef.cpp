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

#include <XBotPlugin/XBotNRTRef.h>
#include <XCM/XBotUtils.h>

REGISTER_XBOT_PLUGIN(XBotNRTRef, XBot::XBotNRTRef)

XBot::XBotNRTRef::XBotNRTRef()
{

}

bool XBot::XBotNRTRef::init_control_plugin( XBot::Handle::Ptr handle)
{
    // get the robot
    _robot = handle->getRobotInterface();

    // create a SubscriberRT for each enabled joint in the robot
    for( int id : _robot->getEnabledJointId() ) {
        _sub_map[id] = XBot::SubscriberRT<XBot::RobotState::pdo_tx>(std::string("rt_in_Motor_id_") + std::to_string(id));
    }
    // and for hands
    for( const auto& h : _robot->getHand() ) {
        int id = h.second->getHandId();
        _sub_map[id] = XBot::SubscriberRT<XBot::RobotState::pdo_tx>(std::string("rt_in_Motor_id_") + std::to_string(id));
    }
    
    // get hand pointers
    for (auto& p: _robot->getHand())
    {
        XBot::Hand::Ptr hand = p.second;
        _hand_map[hand->getHandId()] =  hand;
    }
    
    _ref_map_so["pos_ref_map_so"] = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("pos_ref_map_so");
    _ref_map_so["vel_ref_map_so"] = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("vel_ref_map_so");
    _ref_map_so["tor_ref_map_so"] = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("tor_ref_map_so");
    _ref_map_so["k_ref_map_so"] = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("k_ref_map_so");
    _ref_map_so["d_ref_map_so"] = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("d_ref_map_so");

    return true;
}

void XBot::XBotNRTRef::on_start(double time)
{

}

void XBot::XBotNRTRef::on_stop(double time)
{

}


void XBot::XBotNRTRef::control_loop(double time, double period)
{
    // read from pipes
    
    for( auto& p: _sub_map) {
        if( p.second.read(_pdo_tx) ) {
          
            if( _hand_map[p.first] != nullptr){                
                //  HACK scaling back based on 9.0 max range
                _hand_map[p.first]->grasp(_pdo_tx.pos_ref / 9.0);
            }          
            
            _pos_ref_map[p.first] = _pdo_tx.pos_ref;
            _vel_ref_map[p.first] = _pdo_tx.vel_ref;
            _tor_ref_map[p.first] = _pdo_tx.tor_ref;
            _k_ref_map[p.first] = _pdo_tx.gain_0;
            _d_ref_map[p.first] = _pdo_tx.gain_1;

//             std::cout <<"read --" <<  p.first << " -- " << _pos_ref_map[p.first] << std::endl;
            
        }
        

    }
    
    // update ref in the shared memory
    
    (_ref_map_so.at("pos_ref_map_so")).set(_pos_ref_map);
    (_ref_map_so.at("vel_ref_map_so")).set(_vel_ref_map);
    (_ref_map_so.at("tor_ref_map_so")).set(_tor_ref_map);
    (_ref_map_so.at("k_ref_map_so")).set(_k_ref_map);
    (_ref_map_so.at("d_ref_map_so")).set(_d_ref_map);
    
    
//     for(auto p : *(_ref_map_so.at("pos_ref_map_so"))) {
//         std::cout <<"NRT --" <<  p.first << " -- " << p.second << std::endl;
//     }

}

bool XBot::XBotNRTRef::close(void)
{
    return true;
}

XBot::XBotNRTRef::~XBotNRTRef()
{
    DPRINTF("~XBotNRTRef()\n");
}

