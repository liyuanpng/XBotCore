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

#include <XCM/MessageInterfaces/AdvrJointState.h>
#include <ros/transport_hints.h>
#include <XBotInterface/Utils.h>

#include <XBotInterface/RtLog.hpp>
#include <XBotInterface/SoLib.h>

using XBot::Logger;

REGISTER_SO_LIB_(XBot::AdvrJointState, XBot::GenericJointStateMessage);


XBot::AdvrJointState::AdvrJointState():
    _msg_received(false)
{

}

bool XBot::AdvrJointState::init(const std::string& path_to_config_file, GenericJointStateMessage::Type type)
{

    Logger::info() << "Initializing AdvrJointState message interface!" << Logger::endl();

    std::string robot_name = XBot::ModelInterface::getModel(path_to_config_file)->getUrdf().getName();
    _topic_name = "/xbotcore/" + robot_name + "/joint_states";
    
    ros::NodeHandle nh;
    _sub = nh.subscribe(_topic_name, 1, &AdvrJointState::callback, this);


    // Max number of attempts to connect to /joint_states topic
    int max_connect_attempts = 100;
    double sleep_duration = 0.01;
    int attempt = 0;


    if( type == GenericJointStateMessage::Type::Rx ){
        // Attempt to get a joint state message
        while (!_msg_received && attempt < max_connect_attempts) {
            ros::spinOnce();
            ros::Duration(sleep_duration).sleep();
            attempt++;
        }

        if (!_msg_received) {
            Logger::error() << "No message received on topic " << _topic_name << "! Read() won't work!!" << Logger::endl();
            return false;
        }

    }

    if( type == GenericJointStateMessage::Type::Tx ){

        // Choosing a random joint order for publishing
        auto robot = XBot::RobotInterface::getRobot(path_to_config_file, "xddp_robot");

        _msg.effort.clear();
        _msg.link_position.clear();
        _msg.link_velocity.clear();
        _msg.motor_position.clear();
        _msg.motor_velocity.clear();
        _msg.fault.clear();
        _msg.aux.clear();
        _msg.stiffness.clear();
        _msg.damping.clear();
        _msg.name.clear();
        _msg.temperature.clear();


        for( const std::string& jname : robot->getEnabledJointNames() ){
            _msg.effort.push_back(0);
            _msg.fault.push_back(0);
            _msg.link_position.push_back(0);
            _msg.link_velocity.push_back(0);
            _msg.motor_position.push_back(0);
            _msg.motor_velocity.push_back(0);
            _msg.aux.push_back(0);
            _msg.stiffness.push_back(0);
            _msg.damping.push_back(0);
            _msg.position_reference.push_back(0);
            _msg.velocity_reference.push_back(0);
            _msg.effort_reference.push_back(0);
            _msg.temperature.push_back(0);
            _msg.name.push_back(jname);
        }


    }


    // Populate _idx_map
    {
        int idx = 0;
        for( const std::string& joint_name : _msg.name){
            _id_map[joint_name] = idx;
            idx++;
        }

    }

    // Get a publisher
    _pub = nh.advertise<XBotCore::JointStateAdvr>(_topic_name, 1);

    Logger::success() << "Successfully initialized AdvrJointState message interface!" << Logger::endl();
    
    return true;
}


int XBot::AdvrJointState::getIndex(const std::string& joint_name)
{
    auto it = _id_map.find(joint_name);

    if( it != _id_map.end() ){
        return it->second;
    }
    else{
        Logger::warning() << "in " << __func__ << "! Joint " << joint_name << " is not defined inside the ROS joint state message!" << Logger::endl();
        return -1;
    }
}

double& XBot::AdvrJointState::fault(int index)
{
    return _msg.fault[index];
}


double& XBot::AdvrJointState::aux(int index)
{
    return _msg.aux[index];
}

std::string& XBot::AdvrJointState::aux_name()
{
    return _msg.aux_name;
}

void XBot::AdvrJointState::callback(XBotCore::JointStateAdvrConstPtr msg)
{
    _msg = *msg;
    _msg_received = true;
}

double& XBot::AdvrJointState::damping(int index)
{
    return _msg.damping[index];
}

double& XBot::AdvrJointState::effort(int index)
{
    return _msg.effort[index];
}

double& XBot::AdvrJointState::linkPosition(int index)
{
    return _msg.link_position[index];
}

double& XBot::AdvrJointState::linkVelocity(int index)
{
    return _msg.link_velocity[index];
}

double& XBot::AdvrJointState::motorPosition(int index)
{
    return _msg.motor_position[index];
}

double& XBot::AdvrJointState::motorVelocity(int index)
{
    return _msg.motor_velocity[index];
}

void XBot::AdvrJointState::publish()
{
    _pub.publish(_msg);
}

double& XBot::AdvrJointState::stiffness(int index)
{
    return _msg.stiffness[index];
}

double& XBot::AdvrJointState::temperature(int index)
{
    return _msg.temperature[index];
}

double& XBot::AdvrJointState::position_reference ( int index ) {
    return _msg.position_reference[index];
}

double& XBot::AdvrJointState::velocity_reference ( int index ) {
    return _msg.velocity_reference[index];
}

double& XBot::AdvrJointState::effort_reference ( int index ) {
    return _msg.effort_reference[index];
}