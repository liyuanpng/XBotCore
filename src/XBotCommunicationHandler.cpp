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

#include <XCM/XBotCommunicationHandler.h>


XBot::CommunicationHandler::CommunicationHandler(std::string path_to_config) : _path_to_config(path_to_config)
{

}

void XBot::CommunicationHandler::th_init(void*)
{
    /* Get plugin vector from config file, save switch names, start pipe publishers */
    YAML::Node root_cfg = YAML::LoadFile(_path_to_config);

    if(!root_cfg["XBotRTPlugins"]){
        std::cerr << "ERROR in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << std::endl;
        return;
    }
    else{

        if(!root_cfg["XBotRTPlugins"]["plugins"]){
            std::cerr << "ERROR in " << __func__ << "!XBotRTPlugins node does NOT contain mandatory node plugins!" << std::endl;
        return;
        }
        else{

            for(const auto& plugin : root_cfg["XBotRTPlugins"]["plugins"]){
                _plugin_names.push_back(plugin.as<std::string>());
            }

            std::string communication_plugin_name = "XBotCommunicationPlugin";
            if(std::find(_plugin_names.begin(), _plugin_names.end(), communication_plugin_name) == _plugin_names.end() ) {
                _plugin_names.push_back(communication_plugin_name);
            }
        }

    }

    for(const std::string& name : _plugin_names) {
        std::string switch_name = "xbot_rt_plugin_" + name + "_switch";
        _switch_names.push_back(switch_name);
        _command_pub_vector.push_back(XBot::PublisherNRT<XBot::Command>(switch_name));
    }

    _xddp_handler = std::make_shared<XBot::XBotXDDP>(_path_to_config);
    _xddp_handler->init();

    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint = _xddp_handler;
    std::shared_ptr<XBot::IXBotFT> xbot_ft = _xddp_handler;
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);

    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");


    /* Get a vector of communication interfaces to/from NRT frameworks like ROS, YARP, ... */
#ifdef USE_ROS_COMMUNICATION_INTERFACE
    std::cout << "USE_ROS_COMMUNICATION_INTERFACE found! " << std::endl;
    _master_communication_ifc = std::make_shared<XBot::CommunicationInterfaceROS>(_robot);  // TBD specify the MASTER who can send TX data
    _communication_ifc_vector.push_back( _master_communication_ifc );
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
    std::cout << "USE_YARP_COMMUNICATION_INTERFACE found! " << std::endl;
    communication_ifc_vector.push_back( std::make_shared<XBot::YarpCommunicationInterface>(_robot) );
#endif

    /* Advertise switch ports for all plugins on all frameworks */
    for(auto comm_ifc : _communication_ifc_vector){
        for(const std::string& switch_name : _switch_names){
            comm_ifc->advertiseSwitch(switch_name);
        }
    }

    // set thread name
    name = "ch"; //TBD understand why pthread_setname_np return code error 3
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,1};
    period.task_time = t.task_time;
    period.period = t.period;
    // set scheduler policy
    #ifdef __XENO__
        schedpolicy = SCHED_FIFO;
    #else
        schedpolicy = SCHED_OTHER;
    #endif
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!


}

void XBot::CommunicationHandler::th_loop(void*)
{
    /* Receive commands on switch ports */
    for(auto comm_ifc : _communication_ifc_vector){
        for(int i = 0; i < _plugin_names.size(); i++){
            std::string command;
            if( comm_ifc->receiveFromSwitch(_switch_names[i], command) ){
                _command_pub_vector[i].write(command);
            }
        }
    }

    /* Update XDDP */
     _xddp_handler->update();

    /* Read robot state from RT layer and update robot */
    _robot->sense(false);

    /* Publish robot state to all frameworks */
    for(auto comm_ifc : _communication_ifc_vector){
        comm_ifc->sendRobotState();
    }

    /* Receive commands from the master communication handler,
     * i.e. the only one enabled to send commands to the robot */
    _master_communication_ifc->receiveReference(); // this updates robot

    /* Send received commands to the RT layer */
    _robot->move();
}

XBot::CommunicationHandler::~CommunicationHandler()
{

}

