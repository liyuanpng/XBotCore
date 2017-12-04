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

#ifndef __XBOT_ADVR_JOINT_STATE_H__
#define __XBOT_ADVR_JOINT_STATE_H__

#include <XBotCore/JointStateAdvr.h>
#include <ros/ros.h>
#include <RobotInterfaceROS/GenericJointStateMessage.h>
#include <XBotInterface/RobotInterface.h>

namespace XBot {

    class AdvrJointState : public GenericJointStateMessage {

    public:

        AdvrJointState();

        virtual bool init(const std::string& path_to_config_file, GenericJointStateMessage::Type type);
        virtual int getIndex(const std::string& joint_name);

        virtual double& linkPosition(int index);
        virtual double& motorPosition(int index);

        virtual double& linkVelocity(int index);
        virtual double& motorVelocity(int index);

        virtual double& effort(int index);
        
        virtual double& fault(int index);
        
        virtual double& position_reference ( int index );
        virtual double& velocity_reference ( int index );
        virtual double& effort_reference ( int index );

        virtual double& stiffness(int index);
        virtual double& damping(int index);

        virtual std::string& aux_name();
        virtual double& aux(int index);

        virtual double& temperature(int index);

        virtual void publish();

    private:

        void callback(XBotCore::JointStateAdvrConstPtr msg);

        XBotCore::JointStateAdvr _msg;
        ros::Publisher _pub;
        ros::Subscriber _sub;

        bool _msg_received;

        std::map< std::string, int > _id_map;

        std::string _topic_name;



    };

}



#endif