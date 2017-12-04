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

#ifndef __XBOT_ADVR_COMMAND_MESSAGE_H__
#define __XBOT_ADVR_COMMAND_MESSAGE_H__

#include <XBotCore/CommandAdvr.h>
#include <XBotCore/advr_controller_joint_names.h>
#include <RobotInterfaceROS/GenericControlMessage.h>
#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h>

namespace XBot {

    class CommandAdvr : public GenericControlMessage {

    public:

        virtual bool init(const std::string& path_to_config_file, Type type);

        virtual int getIndex(const std::string& joint_name);

        virtual double& position(int index);
        virtual double& velocity(int index);
        virtual double& effort(int index);
        virtual double& stiffness(int index);
        virtual double& damping(int index);
        virtual double& aux(int index);
        virtual int& seq_id();


        virtual std::string& aux_name();
        virtual void publish();

    private:

        void callback(XBotCore::CommandAdvrConstPtr msg);

        bool service_callback(XBotCore::advr_controller_joint_namesRequest& req, XBotCore::advr_controller_joint_namesResponse& res);

        XBotCore::CommandAdvr _msg;
        ros::Publisher _pub;
        ros::Subscriber _sub;

        ros::ServiceServer _joint_names_srv;
        XBotCore::advr_controller_joint_namesResponse _joint_names_res;

        std::unordered_map<std::string, int> _idx_map;


    };


}





#endif
