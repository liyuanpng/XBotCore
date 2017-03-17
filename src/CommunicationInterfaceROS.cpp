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


#include <XCM/CommunicationInterfaceROS.h>

namespace XBot {

bool CommunicationInterfaceROS::callback(std_srvs::SetBoolRequest& req,
                                         std_srvs::SetBoolResponse& res,
                                         const std::string& port_name)
{
    _msgs.at(port_name) = req.data ? "start" : "stop";
    res.success = true;
    return true;
}


CommunicationInterfaceROS::CommunicationInterfaceROS():
    CommunicationInterface()
{
    int argc = 1;
    char *arg = "dummy_arg";
    char** argv = &arg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "ros_communication_interface");
    }

    _nh = std::make_shared<ros::NodeHandle>();
}

CommunicationInterfaceROS::CommunicationInterfaceROS(XBotInterface::Ptr robot):
    CommunicationInterface(robot),
    _path_to_cfg(robot->getPathToConfig())
{
    int argc = 1;
    char *arg = "dummy_arg";
    char** argv = &arg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "ros_communication_interface");
    }

    _nh = std::make_shared<ros::NodeHandle>();

    load_ros_message_interfaces();
}

void CommunicationInterfaceROS::load_ros_message_interfaces() {

    YAML::Node root_cfg = YAML::LoadFile(_path_to_cfg);
    // TBD check if they exist
    const YAML::Node &ros_interface_root = root_cfg["RobotInterfaceROS"];
    _control_message_type = ros_interface_root["control_message_type"].as<std::string>();
    _jointstate_message_type = ros_interface_root["jointstate_message_type"].as<std::string>();

    const YAML::Node &ctrl_msg_root = root_cfg[_control_message_type];
    _control_message_factory_name = ctrl_msg_root["subclass_factory_name"].as<std::string>();
    _control_message_class_name = ctrl_msg_root["subclass_name"].as<std::string>();
    _control_message_path_to_so = ctrl_msg_root["path_to_shared_lib"].as<std::string>();

    computeAbsolutePath(_control_message_path_to_so,
                        LIB_MIDDLE_PATH,
                        _control_message_path_to_so
                       );

    // Loading the requested control message
    _controlmsg_factory.open( _control_message_path_to_so.c_str(),
                              _control_message_factory_name.c_str());
    if (!_controlmsg_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(_controlmsg_factory.getStatus()).c_str(),
               _controlmsg_factory.getLastNativeError().c_str());
    }
    // open and init control message
    _controlmsg_instance.open(_controlmsg_factory);
    _receive_commands_ok = _controlmsg_instance->init(_path_to_cfg, GenericControlMessage::Type::Rx);
    if(_receive_commands_ok){
        std::cout << "Receive commands from ROS ok!" << std::endl;
    }
    // save pointer to the control message
    _control_message = &_controlmsg_instance.getContent();

    const YAML::Node &jointstate_msg_root = root_cfg[_jointstate_message_type];
    _jointstate_message_factory_name = jointstate_msg_root["subclass_factory_name"].as<std::string>();
    _jointstate_message_class_name = jointstate_msg_root["subclass_name"].as<std::string>();
    _jointstate_message_path_to_so = jointstate_msg_root["path_to_shared_lib"].as<std::string>();

    computeAbsolutePath(_jointstate_message_path_to_so,
                        LIB_MIDDLE_PATH,
                        _jointstate_message_path_to_so
                       );

    // Loading the requested jointstate message
    _jointstatemsg_factory.open( _jointstate_message_path_to_so.c_str(),
                              _jointstate_message_factory_name.c_str());
    if (!_jointstatemsg_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(_jointstatemsg_factory.getStatus()).c_str(),
               _jointstatemsg_factory.getLastNativeError().c_str());
    }
    // open and init jointstate message
    _jointstatemsg_instance.open(_jointstatemsg_factory);
    _send_robot_state_ok = _jointstatemsg_instance->init(_path_to_cfg, GenericJointStateMessage::Type::Tx);
    if(_send_robot_state_ok){
        std::cout << "Send robot state over ROS ok!" << std::endl;
        _send_robot_state_ok = true;
    }
    // save pointer to the jointstate message
    _jointstate_message = &_jointstatemsg_instance.getContent();

    /* Fill maps joint_id -> message indices */

    for( const std::string& joint_name : _robot->getEnabledJointNames() ){
        int id = _robot->getJointByName(joint_name)->getJointId();
        _jointid_to_jointstate_msg_idx[id] = _jointstate_message->getIndex(joint_name);
        _jointid_to_command_msg_idx[id] = _control_message->getIndex(joint_name);;
    }

}
void CommunicationInterfaceROS::sendRobotState()
{
    if( !_send_robot_state_ok ) return;

    _robot->getJointPosition(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
        _jointstate_message->linkPosition(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _robot->getMotorPosition(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
        _jointstate_message->motorPosition(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _robot->getJointVelocity(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
       _jointstate_message->linkVelocity(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _robot->getMotorVelocity(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
        _jointstate_message->motorVelocity(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _robot->getJointEffort(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
        _jointstate_message->effort(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _robot->getTemperature(_joint_id_map);

    for( int id : _robot->getEnabledJointId() ){
        int joint_state_msg_idx = _jointid_to_jointstate_msg_idx.at(id);
        _jointstate_message->temperature(joint_state_msg_idx) = _joint_id_map.at(id);
    }

    _jointstate_message->publish();
}

void CommunicationInterfaceROS::receiveReference()
{
    if( !_receive_commands_ok ) return;

    ros::spinOnce();

    _joint_id_map.clear();

    for( const auto& pair : _jointid_to_command_msg_idx ){
        _joint_id_map[pair.first] = _control_message->position(pair.second);
    }

    _robot->setPositionReference(_joint_id_map);

    _joint_id_map.clear();

    for( const auto& pair : _jointid_to_command_msg_idx ){
        _joint_id_map[pair.first] = _control_message->velocity(pair.second);
    }

    _robot->setVelocityReference(_joint_id_map);

    _joint_id_map.clear();

    for( const auto& pair : _jointid_to_command_msg_idx ){
        _joint_id_map[pair.first] = _control_message->effort(pair.second);
    }

    _robot->setEffortReference(_joint_id_map);

    _joint_id_map.clear();

    for( const auto& pair : _jointid_to_command_msg_idx ){
        _joint_id_map[pair.first] = _control_message->stiffness(pair.second);
    }

    _robot->setStiffness(_joint_id_map);

    _joint_id_map.clear();

    for( const auto& pair : _jointid_to_command_msg_idx ){
        _joint_id_map[pair.first] = _control_message->damping(pair.second);
    }

    _robot->setDamping(_joint_id_map);


}

bool CommunicationInterfaceROS::advertiseSwitch(const std::string& port_name)
{
    if( _services.count(port_name) > 0 ){
        return false;
    }

    _services[port_name] = _nh->advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>
                                    (port_name,
                                     boost::bind(&CommunicationInterfaceROS::callback,
                                                 this,
                                                 _1, _2, port_name)
                                     );
    _msgs[port_name] = "";

    std::cout << "Advertised service " << port_name << std::endl;

    return true;
}

bool CommunicationInterfaceROS::receiveFromSwitch(const std::string& port_name, std::string& message)
{
    ros::spinOnce();

    auto it = _msgs.find(port_name);

    if( it == _msgs.end() ) return false;

    message = it->second;
    if( message != "" ){
        it->second = "";
        return true;
    }
    else {
        return false;
    }
}

bool CommunicationInterfaceROS::computeAbsolutePath (  const std::string& input_path,
                                                 const std::string& middle_path,
                                                 std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}


}