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

#include <XCM/XBotESCUtils.h>

XBot::ESCUtils::ESCUtils(XBot::RobotInterface::Ptr robot) : _robot(robot)
{

}

bool XBot::ESCUtils::setReferenceFromRobotStateTX(const std::map< int, XBot::RobotState::pdo_tx >& pdo_tx)
{
    _joint_map.clear();
    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.pos_ref;
    }
    _robot->setPositionReference(_joint_map);
    
    _joint_map.clear();
    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.vel_ref;
    }
    _robot->setVelocityReference(_joint_map);
    
    _joint_map.clear();
    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.tor_ref;
    }
    _robot->setEffortReference(_joint_map);
}

bool XBot::ESCUtils::setRobotStateFromRobotInterface(std::map< int, XBot::RobotState >& pdo)
{
    _joint_map.clear();
    _robot->getJointPosition(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateRX.link_pos = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getMotorPosition(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateRX.motor_pos = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getJointVelocity(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateRX.link_vel = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getMotorVelocity(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateRX.motor_vel = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getJointEffort(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateRX.torque = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getPositionReference(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateTX.pos_ref = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getVelocityReference(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateTX.vel_ref = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getEffortReference(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateTX.tor_ref = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getStiffness(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateTX.gain_0 = _joint_map.at(pair.first);
    }
    
    _joint_map.clear();
    _robot->getDamping(_joint_map);
    for( const auto& pair : _joint_map ){
        pdo[pair.first].RobotStateTX.gain_1 = _joint_map.at(pair.first);
    }
}

bool XBot::ESCUtils::setRobotFTFromRobotInterface(std::map< int, XBot::RobotFT::pdo_rx >& ft)
{
    for( const auto& pair : _robot->getForceTorque() ){
        int id = pair.second->getSensorId();
        _robot->getForceTorque(id)->getWrench(_wrench);
        ft[id].force_X  = _wrench(0);
        ft[id].force_Y  = _wrench(1);
        ft[id].force_Z  = _wrench(2);
        ft[id].torque_X = _wrench(3);
        ft[id].torque_Y = _wrench(4);
        ft[id].torque_Z = _wrench(5);
    }

}


