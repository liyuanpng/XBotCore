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

#include <XCM/XBotUtils.h>

XBot::ESCUtils::ESCUtils(XBot::RobotInterface::Ptr robot) :
    _robot(robot)
{
    _robot->getJointPosition(_q);
    _robot->getMotorPosition(_mq);
    _robot->getJointVelocity(_qdot);
    _robot->getMotorVelocity(_mqdot);
    _robot->getJointEffort(_tau);
    _robot->getStiffness(_k);
    _robot->getDamping(_d);
    _robot->getPositionReference(_qref);
    _robot->getVelocityReference(_qdotref);
    _robot->getEffortReference(_tauref);
}

bool XBot::ESCUtils::setReferenceFromRobotStateTX(const std::map< int, XBot::RobotState::pdo_tx >& pdo_tx)
{
    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.pos_ref;
    }
    _robot->setPositionReference(_joint_map);

    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.vel_ref;
    }
    _robot->setVelocityReference(_joint_map);

    for( const auto& pair : pdo_tx ){
        _joint_map[pair.first] = pair.second.tor_ref;
    }
    _robot->setEffortReference(_joint_map);
}

bool XBot::ESCUtils::setRobotStateFromRobotInterface(std::map< int, XBot::RobotState >& pdo)
{

    _robot->getJointPosition(_q);
    _robot->getMotorPosition(_mq);
    _robot->getJointVelocity(_qdot);
    _robot->getMotorVelocity(_mqdot);
    _robot->getJointEffort(_tau);
    _robot->getStiffness(_k);
    _robot->getDamping(_d);
    _robot->getPositionReference(_qref);
    _robot->getVelocityReference(_qdotref);
    _robot->getEffortReference(_tauref);

    for(int i = 0; i < _robot->getJointNum(); i++){
        const auto& joint_ids = _robot->getEnabledJointId();
        auto& robot_state = pdo[joint_ids[i]];

        robot_state.RobotStateRX.link_pos = _q[i];
        robot_state.RobotStateRX.motor_pos = _mq[i];
        robot_state.RobotStateRX.link_vel = _qdot[i];
        robot_state.RobotStateRX.motor_vel = _mqdot[i];
        robot_state.RobotStateRX.torque = _tau[i];

        robot_state.RobotStateTX.gain_0 = _k[i];
        robot_state.RobotStateTX.gain_1 = _d[i];
        robot_state.RobotStateTX.pos_ref = _qref[i];
        robot_state.RobotStateTX.vel_ref = _qdotref[i];
        robot_state.RobotStateTX.tor_ref = _tauref[i];

    }
}

bool XBot::ESCUtils::setRobotFTFromRobotInterface(std::map< int, XBot::RobotFT::pdo_rx >& ft)
{
    for( const auto& pair : _robot->getForceTorque() ){
        int id = pair.second->getSensorId();
        pair.second->getWrench(_wrench);
        ft[id].force_X  = _wrench(0);
        ft[id].force_Y  = _wrench(1);
        ft[id].force_Z  = _wrench(2);
        ft[id].torque_X = _wrench(3);
        ft[id].torque_Y = _wrench(4);
        ft[id].torque_Z = _wrench(5);
    }

}

bool XBot::ESCUtils::setRobotIMUFromRobotInterface(std::map< int, XBot::RobotIMU::pdo_rx >& imu)
{
    for( const auto& pair : _robot->getImu() ){
            int id = pair.second->getSensorId();
            pair.second->getImuData(_orientation, _lin_acc, _ang_vel); 
            
            imu[id].quat_X  = _orientation.x();
            imu[id].quat_Y  = _orientation.y();
            imu[id].quat_Z  = _orientation.z();
            imu[id].quat_W  = _orientation.w();
            
            imu[id].lin_acc_X = _lin_acc(0);
            imu[id].lin_acc_Y = _lin_acc(1);
            imu[id].lin_acc_Z = _lin_acc(2);
            
            imu[id].ang_vel_X = _ang_vel(0);
            imu[id].ang_vel_Y = _ang_vel(1);
            imu[id].ang_vel_Z = _ang_vel(2);
        }
}


