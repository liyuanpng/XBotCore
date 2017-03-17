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


#ifndef __XBOT_COMMUNICATION_INTERFACE_ROS_H__
#define __XBOT_COMMUNICATION_INTERFACE_ROS_H__

#include <XCM/XBotCommunicationInterface.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <boost/bind.hpp>
#include <RobotInterfaceROS/GenericControlMessage.h>
#include <RobotInterfaceROS/GenericJointStateMessage.h>

namespace XBot {

class CommunicationInterfaceROS : public CommunicationInterface {

public:

    CommunicationInterfaceROS();
    CommunicationInterfaceROS(XBotInterface::Ptr robot);

    virtual void sendRobotState();
    virtual void receiveReference();

    virtual bool advertiseSwitch(const std::string& port_name);
    virtual bool receiveFromSwitch(const std::string& port_name, std::string& message);

protected:

private:

    bool callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res, const std::string& port_name);

    void load_ros_message_interfaces();

    static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& middle_path,
                                       std::string& absolute_path );

    bool _send_robot_state_ok, _receive_commands_ok;

    std::string _path_to_cfg;

    JointIdMap _joint_id_map;

    shlibpp::SharedLibraryClass<GenericJointStateMessage> _jointstatemsg_instance;
    shlibpp::SharedLibraryClassFactory<GenericJointStateMessage> _jointstatemsg_factory;
    GenericJointStateMessage::Ptr _jointstate_message;

    std::string _jointstate_message_type,
                _jointstate_message_factory_name,
                _jointstate_message_class_name,
                _jointstate_message_path_to_so;

    shlibpp::SharedLibraryClass<GenericControlMessage> _controlmsg_instance;
    shlibpp::SharedLibraryClassFactory<GenericControlMessage> _controlmsg_factory;
    GenericControlMessage::Ptr _control_message;

    std::string _control_message_type,
                _control_message_factory_name,
                _control_message_class_name,
                _control_message_path_to_so;

    std::unordered_map<int, int> _jointid_to_command_msg_idx;
    std::unordered_map<int, int> _jointid_to_jointstate_msg_idx;

    std::shared_ptr<ros::NodeHandle> _nh;

    std::map<std::string, ros::ServiceServer> _services;
    std::map<std::string, std::string> _msgs;



};


}

#endif //__XBOT_COMMUNICATION_INTERFACE_ROS_H__