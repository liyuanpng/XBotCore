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

#include <XCM/XBotCommunicationInterface.h>
#include <XCM/XBotXDDP.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <csignal>

#ifdef USE_ROS_COMMUNICATION_INTERFACE
#include <XCM/CommunicationInterfaceROS.h>
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
#include <XCM/CommunicationInterfaceYARP.h>
#endif

volatile sig_atomic_t running = 1;

void sigint_handler(int s){
    running = 0;
}

int main(int argc, char ** argv){

    /* Get config file from command line */
    std::string path_to_config_file = argv[1];

    /* Get plugin vector from config file, save switch names, start pipe publishers */
    std::vector<std::string> plugin_names;

    YAML::Node root_cfg = YAML::LoadFile(path_to_config_file);

    if(!root_cfg["XBotRTPlugins"]){
        std::cerr << "ERROR in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << std::endl;
        return false;
    }
    else{

        if(!root_cfg["XBotRTPlugins"]["plugins"]){
            std::cerr << "ERROR in " << __func__ << "!XBotRTPlugins node does NOT contain mandatory node plugins!" << std::endl;
        return false;
        }
        else{

            for(const auto& plugin : root_cfg["XBotRTPlugins"]["plugins"]){
                plugin_names.push_back(plugin.as<std::string>());
            }
        }

    }

    std::vector<std::string> switch_names;
    std::vector<XBot::PublisherNRT<XBot::Command>> command_pub_vector;
    for(const std::string& name : plugin_names){
        std::string switch_name = "xbot_rt_plugin_" + name + "_switch";
        switch_names.push_back(switch_name);
        command_pub_vector.push_back(XBot::PublisherNRT<XBot::Command>(switch_name));
    }


    /* By building a RobotinterfaceXBotRT we are able to make use of the
     * respectable XBotXDDP! */
    XBot::XBotXDDP::Ptr xddp_handler = std::make_shared<XBot::XBotXDDP>(path_to_config_file);
//
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint = xddp_handler;
    std::shared_ptr<XBot::IXBotFT> xbot_ft = xddp_handler;
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);

    auto robot = XBot::RobotInterface::getRobot(path_to_config_file, anymap, "XBotRT");


    /* Get a vector of communication interfaces to/from NRT frameworks like ROS, YARP, ... */
    std::vector<XBot::CommunicationInterface::Ptr> communication_ifc_vector;
    XBot::CommunicationInterface::Ptr master_communication_ifc;

#ifdef USE_ROS_COMMUNICATION_INTERFACE
    master_communication_ifc = std::make_shared<XBot::CommunicationInterfaceROS>(robot); 
    communication_ifc_vector.push_back( master_communication_ifc );
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
    communication_ifc_vector.push_back( std::make_shared<XBot::YarpCommunicationInterface>(robot) );
#endif

    /* Register SIGINT handler AFTER FRAMEWORK HAVE BEEN INITILIZED */
    signal(SIGINT, sigint_handler);

    /* Advertise switch ports for all plugins on all frameworks */
    for(auto comm_ifc : communication_ifc_vector){
        for(const std::string& switch_name : switch_names){
            comm_ifc->advertiseSwitch(switch_name);
        }
    }

    /* Main loop */

    while(running){

        /* Receive commands on switch ports */
        for(auto comm_ifc : communication_ifc_vector){
            for(int i = 0; i < plugin_names.size(); i++){
                std::string command;
                if( comm_ifc->receiveFromSwitch(switch_names[i], command) ){
                    command_pub_vector[i].write(command);
                }
            }
        }

        /* Read robot state from RT layer and update robot */
        robot->sense(false);

        /* Publish robot state to all frameworks */
        for(auto comm_ifc : communication_ifc_vector){
            comm_ifc->sendRobotState();
        }

        /* Receive commands from the master communication handler,
         * i.e. the only one enabled to send commands to the robot */
        master_communication_ifc->receiveReference(); // this updates robot

        /* Send received commands to the RT layer */
        robot->move();

        usleep(5000);
    }



}