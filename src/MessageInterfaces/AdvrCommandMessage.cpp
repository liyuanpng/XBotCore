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

#include <XCM/MessageInterfaces/AdvrCommandMessage.h>
#include <ros/transport_hints.h> 

SHLIBPP_DEFINE_SHARED_SUBCLASS(advrcommandmessage_control_message, XBot::CommandAdvr, XBot::GenericControlMessage);


void XBot::CommandAdvr::callback(XCM::CommandAdvrConstPtr msg)
{
    for(int i = 0; i < msg->name.size(); i++){

        int idx = getIndex(msg->name[i]);

        if(idx < 0){
            std::cerr << "ERROR while parsing CommandAdvr message: joint " << msg->name[i] << " undefined" << std::endl;
            continue;
        }

        if(msg->aux.size() > i){
            _msg.aux[idx] = msg->aux[i];
        }

        _msg.aux_name = msg->aux_name;

        if(msg->damping.size() > i){
            _msg.damping[idx] = msg->damping[i];
        }

        if(msg->effort.size() > i){
            _msg.effort[idx] = msg->effort[i];
        }

        if(msg->position.size() > i){
            _msg.position[idx] = msg->position[i];
        }

        if(msg->stiffness.size() > i){
            _msg.stiffness[idx] = msg->stiffness[i];
        }

        if(msg->velocity.size() > i){
            _msg.velocity[idx] = msg->velocity[i];
        }
        
        _msg.seq_id++;
    }
}

bool XBot::CommandAdvr::service_callback(XCM::advr_controller_joint_namesRequest& req,
                                         XCM::advr_controller_joint_namesResponse& res)
{
    res = _joint_names_res;
    return true;
}


bool XBot::CommandAdvr::init(const std::string& path_to_config_file, XBot::GenericControlMessage::Type type)
{

    std::cout << "Initializing CommandAdvr message interface" << std::endl;
    
    

    YAML::Node root_cfg = YAML::LoadFile(path_to_config_file);

    // TBD check if they exist
    const YAML::Node &ctrl_msg_root = root_cfg["AdvrCommandMessage"];
    std::string joint_service_name = ctrl_msg_root["joint_service_name"].as<std::string>();
    std::string command_topic_name = ctrl_msg_root["command_topic_name"].as<std::string>();

    ros::NodeHandle nh;

    
    
    if( type == XBot::GenericControlMessage::Type::Rx ) {
        
        _sub = nh.subscribe(command_topic_name, 1, &XBot::CommandAdvr::callback, this, ros::TransportHints().tcpNoDelay());

        auto robot = XBot::RobotInterface::getRobot(path_to_config_file);
        robot->sense();

        XBot::JointNameMap _joint_pos, _joint_stiffness, _joint_damping;
        robot->getJointPosition(_joint_pos);
        robot->getStiffness(_joint_stiffness);
        robot->getDamping(_joint_damping);

        for( const std::string& jname : robot->getEnabledJointNames() ){

            _joint_names_res.name.push_back(jname);
            _msg.name.push_back(jname);
            _msg.aux.push_back(0);
            _msg.damping.push_back(_joint_damping.at(jname));
            _msg.effort.push_back(0);
            _msg.position.push_back(_joint_pos.at(jname));
            _msg.stiffness.push_back(_joint_stiffness.at(jname));
            _msg.velocity.push_back(0);
        }
        
        _msg.seq_id = 0;

        _joint_names_srv = nh.advertiseService(joint_service_name, &XBot::CommandAdvr::service_callback, this);
    }


    if( type == XBot::GenericControlMessage::Type::Tx ) {
        
        _pub = nh.advertise<XCM::CommandAdvr>(command_topic_name, 1);


        ros::ServiceClient client = nh.serviceClient<XCM::advr_controller_joint_names>(joint_service_name);
        XCM::advr_controller_joint_namesRequest req;

        if (!client.call(req, _joint_names_res)) {
            std::cerr << "ERROR service client " << joint_service_name << " not available! Write() won't work!!" << std::endl;
            return false;
        }

        for( const std::string& jname : _joint_names_res.name ){
            _msg.name.push_back(jname);
            _msg.aux.push_back(0);
            _msg.damping.push_back(0);
            _msg.effort.push_back(0);
            _msg.position.push_back(0);
            _msg.stiffness.push_back(0);
            _msg.velocity.push_back(0);
        }
        
        _msg.seq_id = 0;
    }

    // Populate _idx_map
    {
        int idx = 0;
        for( const std::string& joint_name : _joint_names_res.name){
            _idx_map[joint_name] = idx;
            idx++;
        }

    }
    
    
    
    return true;

}

int XBot::CommandAdvr::getIndex(const std::string& joint_name)
{
    auto it = _idx_map.find(joint_name);

    if( it != _idx_map.end() ){
        return it->second;
    }
    else{
        std::cerr << "WARNING in " << __func__ << "! Joint " << joint_name << " is not defined inside the ROS controller!" << std::endl;
        return -1;
    }

}

double& XBot::CommandAdvr::aux(int index)
{
    return _msg.aux[index];
}

std::string& XBot::CommandAdvr::aux_name()
{
    return _msg.aux_name;
}

double& XBot::CommandAdvr::damping(int index)
{
    return _msg.damping[index];
}

double& XBot::CommandAdvr::effort(int index)
{
    return _msg.effort[index];
}

double& XBot::CommandAdvr::position(int index)
{
    return _msg.position[index];
}

double& XBot::CommandAdvr::stiffness(int index)
{
    return _msg.stiffness[index];
}

double& XBot::CommandAdvr::velocity(int index)
{
    return _msg.velocity[index];
}

int& XBot::CommandAdvr::seq_id()
{
    return _msg.seq_id;
}

void XBot::CommandAdvr::publish()
{
    _pub.publish(_msg);
}
