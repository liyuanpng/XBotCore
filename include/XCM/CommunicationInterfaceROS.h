/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore, Giuseppe Rigano
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it, giuseppe.rigano@iit.it
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
#include <robot_state_publisher/robot_state_publisher.h>

#include <XBotCore/cmd_service.h>
#include <XBotCore/status_service.h>
#include <std_msgs/Float64.h>

namespace XBot {

class CommunicationInterfaceROS : public CommunicationInterface {

public:

    CommunicationInterfaceROS();
    CommunicationInterfaceROS(XBotInterface::Ptr robot, 
                              XBot::XBotXDDP::Ptr xddp_handler,
                              XBot::IXBotJoint::Ptr xbot_joint = nullptr
                             );

    virtual void sendRobotState();
    virtual void receiveReference();
    virtual void resetReference();

    virtual bool advertiseSwitch(const std::string& port_name);
    virtual bool receiveFromSwitch(const std::string& port_name, std::string& message);

    virtual bool advertiseCmd(const std::string& port_name);
    virtual bool receiveFromCmd(const std::string& port_name, std::string& message);  // TBD template message

    virtual bool advertiseMasterCommunicationInterface();
    virtual bool receiveMasterCommunicationInterface(std::string& framework_name);

    virtual void advertiseStatus(const std::string& plugin_name);
    virtual bool setPluginStatus(const std::string& plugin_name, const std::string& status);
    virtual std::string getPluginStatus(const std::string& plugin_name);

protected:

private:

    bool callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res, const std::string& port_name);
    bool callback_cmd(::XBotCore::cmd_serviceRequest& req, ::XBotCore::cmd_serviceResponse& res, const std::string& port_name);
    bool callback_master_communication_iface(::XBotCore::cmd_serviceRequest& req, ::XBotCore::cmd_serviceResponse& res, const std::string& port_name);
    bool callback_status(::XBotCore::status_serviceRequest& req, ::XBotCore::status_serviceResponse& res, const std::string& plugin_name);
    
    bool callback_hand(const std_msgs::Float64::ConstPtr& msg, int hand_id);

    void load_ros_message_interfaces();

    void load_robot_state_publisher();

    static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& middle_path,
                                       std::string& absolute_path );

    bool _send_robot_state_ok, _receive_commands_ok, _publish_tf;

    std::string _path_to_cfg;

    JointIdMap _joint_id_map;
    JointNameMap _joint_name_map;

    std::map<std::string, ros::ServiceServer> _status_services;
    std::map<std::string, std::string> _plugin_status_map;

    std::shared_ptr<GenericJointStateMessage> _jointstate_message;

    std::string _jointstate_message_type,
                _jointstate_message_factory_name,
                _jointstate_message_class_name,
                _jointstate_message_path_to_so;

    std::shared_ptr<GenericControlMessage> _control_message;

    std::string _control_message_type,
                _control_message_factory_name,
                _control_message_class_name,
                _control_message_path_to_so;

    std::unordered_map<int, int> _jointid_to_command_msg_idx;
    std::unordered_map<int, int> _jointid_to_jointstate_msg_idx;

    std::shared_ptr<ros::NodeHandle> _nh;

    std::map<std::string, ros::ServiceServer> _services;
    std::map<std::string, std::string> _msgs;

    std::shared_ptr<robot_state_publisher::RobotStatePublisher> _robot_state_pub;
    std::string _tf_prefix, _urdf_param_name;

    std::map<int, ros::Publisher> _imu_pub_map;
    std::map<int, ros::Publisher> _ft_pub_map;
    
    std::map<int, ros::Publisher> _hand_pub_map;
    std::map<int, ros::Subscriber> _hand_sub_map;
    std::map<int, double> _hand_value_map;
    
    int current_seq_id = 0;


};


}

#endif //__XBOT_COMMUNICATION_INTERFACE_ROS_H__