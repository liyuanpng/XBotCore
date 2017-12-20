/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <XCM/XBotCommunicationHandler.h>
#include <XCM/IOPluginFactory.h>

#include <XBotInterface/RtLog.hpp>

#include <dirent.h>

using XBot::Logger;



XBot::CommunicationHandler::CommunicationHandler(std::string path_to_config, 
                                                 SharedMemory::Ptr shmem, 
                                                 Options opt
                                                 ) :
    _path_to_config(path_to_config),
    _master_communication_ifc(nullptr),
    loadWebServer(true),
    _shmem(shmem),
    _options(opt)
{
    // set thread name
    name = "XBOT_COMMHANDLER"; //TBD understand why pthread_setname_np return code error 3
    
    // Set thread period
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0, _options.comm_handler_period_us};
    period.task_time = t.task_time;
    period.period = t.period;
    
    // Set scheduler policy
    schedpolicy = SCHED_OTHER;
    
    // Set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    // Init ROS
    int argc = 1;
    const char * argvv = "CommHandler";
    char ** argv = (char **)(&argvv);
    
    ros::init(argc, argv, "CommHandler", ros::init_options::NoSigintHandler);
    
    // Construct RosHandle and push it to shared memory
    _roshandle = std::make_shared<RosUtils::RosHandle>();
    _roshandle_shobj = _shmem->getSharedObject<RosUtils::RosHandle::Ptr>("ros_handle");
    _roshandle_shobj.set(_roshandle);
    Logger::info() << "Set RosHandle! " << _roshandle << Logger::endl();
    
}

void XBot::CommunicationHandler::th_init(void*)
{
    
    const char* env_user = std::getenv("USER");
    Logger::info() << "USER is: " << env_user << Logger::endl();
    std::string folder = std::string("/tmp/")+env_user;
    DIR* dir = opendir(folder.c_str());
    if (dir)
    {
        /* Directory exists. */
        closedir(dir);
    }
    else{
      const int dir_err = mkdir(folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (-1 == dir_err)
      {
        Logger::error() << "Unable to create user directory!" << Logger::endl();
        exit(1);
      }
    }
      
    // check that config file exists
    std::ifstream fin(_path_to_config);
    if (fin.fail()) {
        Logger::error() << "in " << __func__ << "! Can NOT open config file " << _path_to_config << "!" << Logger::endl();
        exit(0);
    }
    
    /* Get plugin vector from config file, save switch names, start pipe publishers */
    YAML::Node root_cfg = YAML::LoadFile(_path_to_config);

    if(!root_cfg["XBotRTPlugins"]){
        Logger::error() << "in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << Logger::endl();
        return;
    }
    else{

        if(!root_cfg["XBotRTPlugins"]["plugins"]){
            Logger::error() << "in " << __func__ << "! XBotRTPlugins node does NOT contain mandatory node plugins!" << Logger::endl();
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

        if(!root_cfg["XBotRTPlugins"]["io_plugins"]){
            Logger::warning() << "in " << __func__ << "! XBotRTPlugins node does NOT contain mandatory node io_plugins!" << Logger::endl();
        }
        else{

            for(const auto& plugin : root_cfg["XBotRTPlugins"]["io_plugins"]){
                _io_plugin_names.push_back(plugin.as<std::string>());
            }

        }
    }

    if(!root_cfg["MasterCommunicationInterface"]) {
        Logger::error() << "in " << __func__ << "! Config file does NOT contain mandatory node MasterCommunicationInterface!" << Logger::endl();
    }
    else {
        if(!root_cfg["MasterCommunicationInterface"]["framework_name"]){
            Logger::error() << "in " << __func__ << "! MasterCommunicationInterface node does NOT contain mandatory node framework_name!" << Logger::endl();
            return;
        }
        else{
            _master_communication_interface_name = root_cfg["MasterCommunicationInterface"]["framework_name"].as<std::string>();
        }
        
        _enable_ref_read = true;
        
        if(!root_cfg["MasterCommunicationInterface"]["enable_ref_read"]){
            Logger::warning(Logger::Severity::LOW) << "in " << __func__ << "! MasterCommunicationInterface node does NOT contain optional node enable_ref_read: I will assume it to TRUE" << Logger::endl();
        }
        else{
            _enable_ref_read = root_cfg["MasterCommunicationInterface"]["enable_ref_read"].as<bool>();
        }
    }
    


    int plugin_idx = 0;
    for(const std::string& name : _plugin_names) {
        std::string switch_name = name + "_switch";
        _switch_names.push_back(switch_name);
        _switch_pub_vector.push_back(XBot::PublisherNRT<XBot::Command>(switch_name));

        std::string command_name = name + "_cmd";
        _command_names.push_back(command_name);
        _command_pub_vector.push_back(XBot::PublisherNRT<XBot::Command>(command_name));

        _status_sub_vector.push_back(XBot::SubscriberNRT<XBot::Command>(name + "_status"));
        // save XBotCommunicationPlugin index
        xbot_communication_idx = plugin_idx;
        plugin_idx++;
    }


    _xddp_handler = std::make_shared<XBot::XBotXDDP>(_path_to_config);
    _xddp_handler->init();

    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint = _xddp_handler;
    std::shared_ptr<XBot::IXBotFT> xbot_ft = _xddp_handler;
    std::shared_ptr<XBot::IXBotIMU> xbot_imu = _xddp_handler;
    std::shared_ptr<XBot::IXBotHand> xbot_hand = _xddp_handler;

    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    (*anymap)["EnableReferenceReading"] = boost::any(_enable_ref_read);

    _robot = XBot::RobotInterface::getRobot(_path_to_config, "xddp_robot", anymap, "XBotRT");

    _logger = XBot::MatLogger::getLogger("/tmp/CommunicationHandler_log");

    // update robot
    _robot->sense();


    /* Get a vector of communication interfaces to/from NRT frameworks like ROS, YARP, ... */
#ifdef USE_ROS_COMMUNICATION_INTERFACE
    Logger::info() << "USE_ROS_COMMUNICATION_INTERFACE found! " << Logger::endl();
    _ros_communication = std::make_shared<XBot::CommunicationInterfaceROS>(_robot, _xddp_handler, xbot_joint);
    _communication_ifc_vector.push_back( _ros_communication );

    if ( _master_communication_interface_name == "ROS" ||
         _master_communication_interface_name == "ros"
    ) {
        _master_communication_ifc = _ros_communication;
    }

#endif
    
    const YAML::Node &web_server = root_cfg["WebServer"]; 
    if(web_server){
       std::string load = web_server["enable"].as<std::string>();
       if (load.compare("true") == 0 ) 
         loadWebServer = true;
       else
         loadWebServer = false;       
    }
    
    /********************************WEB INTERFACE********************************************/
    if (loadWebServer) {
      _web_communication = CommunicationInterfaceFactory::getFactory("libwebserver", "WEB_SERVER",_robot);
      if(_web_communication){
        _communication_ifc_vector.push_back( _web_communication );
      }
    }
    /****************************************************************************************/
    
    /********************************ROS INTERFACE********************************************/
//     _ros_communication = CommunicationInterfaceFactory::getFactory("libros", "ROS",_robot);
//     if(_ros_communication){
//       _communication_ifc_vector.push_back( _ros_communication );
//     }
    /****************************************************************************************/
    
    /********************************YARP INTERFACE********************************************/
//     _yarp_communication = CommunicationInterfaceFactory::getFactory("libyarp", "YARP",_robot);
//     if(_yarp_communication){
//       _communication_ifc_vector.push_back( _yarp_communication );
//     }
    /****************************************************************************************/
    
#ifdef USE_YARP_COMMUNICATION_INTERFACE
    Logger::info() << "USE_YARP_COMMUNICATION_INTERFACE found! " << Logger::endl();
    _yarp_communication = std::make_shared<XBot::CommunicationInterfaceYARP>(_robot);
    _communication_ifc_vector.push_back( _yarp_communication );

    if ( _master_communication_interface_name == "YARP" ||
         _master_communication_interface_name == "yarp"
    ) {
        _master_communication_ifc = _yarp_communication;
    }

#endif 

    // check on master communication interface
    if( _master_communication_ifc == nullptr ) {
        Logger::error() << "in " << __func__ << "! Master Communication Interface specified in the config file but "
                                             << "not matching with the current NRT frameworks installed in the system" << Logger::endl();
        return;
    }

    /* Load IO plugins */
    for(const std::string& name : _io_plugin_names) {
        std::shared_ptr<XBot::IOPlugin> plugin_ptr = IOPluginFactory::getFactory("lib"+name, name);
        _io_plugin_ptr.push_back(plugin_ptr);
        Logger::info(Logger::Severity::MID, "Initializing IO plugin %s...\n", name.c_str());
        plugin_ptr->init(_path_to_config, _shmem);
        Logger::success(Logger::Severity::MID, "Initialized IO plugin %s\n!", name.c_str());
    }

    /* Advertise switch/cmd ports for all plugins on all frameworks */
    for(auto comm_ifc : _communication_ifc_vector){

        /* Advertise swtich port for Master Communication Interface */
        comm_ifc->advertiseMasterCommunicationInterface();

        for(const std::string& switch_name : _switch_names){
            comm_ifc->advertiseSwitch(switch_name);
        }

        for(const std::string& cmd_name : _command_names){
            comm_ifc->advertiseCmd(cmd_name);
        }

        for(const std::string& pl_name : _plugin_names){
            comm_ifc->advertiseStatus(pl_name);
        }
    }

    /* Initialize references */
    Eigen::VectorXd q0;
    _robot->getMotorPosition(q0);
    _robot->setPositionReference(q0);

}

void XBot::CommunicationHandler::th_loop(void*)
{
    /* Receive Master Communication Interface to switch NRT framework at runtime */

    for(auto comm_ifc : _communication_ifc_vector) {

        std::string master;

        if( comm_ifc->receiveMasterCommunicationInterface(master) ) {

            if ( master == "ROS" ||
                 master == "ros"
            ) {
                Logger::info(Logger::Severity::HIGH) << "Switching to ROS Master Communication Interface" << Logger::endl();

#ifdef USE_ROS_COMMUNICATION_INTERFACE
                _master_communication_ifc = _ros_communication;
                // HACK restarting XBotCommunicationPlugin
                std::string cmd = "stop";
                _switch_pub_vector[xbot_communication_idx].write(cmd);
                sleep(1);
                cmd = "start";
                _switch_pub_vector[xbot_communication_idx].write(cmd);
#else
                Logger::error() << "ROS Master Communication Interface not compiled" << Logger::endl();
#endif
            }
            
            else if ( master == "WEB" ||
                 master == "web"
            ) {
                Logger::info(Logger::Severity::HIGH) << "Switching to WEB Master Communication Interface" << Logger::endl();

                _master_communication_ifc = _web_communication;
                // HACK restarting XBotCommunicationPlugin
                std::string cmd = "stop";
                _switch_pub_vector[xbot_communication_idx].write(cmd);
                sleep(1);
                cmd = "start";
                _switch_pub_vector[xbot_communication_idx].write(cmd);

                
            }

            else if ( master == "YARP" ||
                      master == "yarp"
            ) {
                Logger::info(Logger::Severity::HIGH) << "Switching to YARP Master Communication Interface" << Logger::endl();

#ifdef USE_YARP_COMMUNICATION_INTERFACE
                _master_communication_ifc = _yarp_communication;
                // HACK restarting XBotCommunicationPlugin
                std::string cmd = "stop";
                _switch_pub_vector[xbot_communication_idx].write(cmd);
                sleep(1);
                cmd = "start";
                _switch_pub_vector[xbot_communication_idx].write(cmd);
#else
                Logger::error() << "YARP Master Communication Interface not compiled" << Logger::endl();
#endif
            }
        }
    }

    /* Receive commands on switch ports */
    for(auto comm_ifc : _communication_ifc_vector) {
        for(int i = 0; i < _plugin_names.size(); i++){
            std::string command;
            if( comm_ifc->receiveFromSwitch(_switch_names[i], command) ){
                _switch_pub_vector[i].write(command);
            }
            if( comm_ifc->receiveFromCmd(_command_names[i], command) ){
                _command_pub_vector[i].write(command);
            }

            XBot::Command status;
            if( _status_sub_vector[i].read(status) ){
                comm_ifc->setPluginStatus(_plugin_names[i], status.str());
            }
        }
    }

    /* Update XDDP (receive from RT) */
     _xddp_handler->updateRX();

    /* Read robot state from RT layer and update robot */
    _robot->sense(false);

    /* Publish robot state to all frameworks */
    for(auto comm_ifc : _communication_ifc_vector){
        comm_ifc->sendRobotState();
    }

    /* Receive commands from the master communication handler,
     * i.e. the only one enabled to send commands to the robot */
    _master_communication_ifc->receiveReference(); // this updates robot
    
    /* Run external plugins */
    for( std::shared_ptr<XBot::IOPlugin> io_plugin_ptr : _io_plugin_ptr ){
        if(io_plugin_ptr) io_plugin_ptr->run();
    }

    /* Send received commands to the RT layer */
    _robot->move();
    
    /* Update XDDP (send to RT) */
    _xddp_handler->updateTX();
    
    /* Publish all pending ROS messages */
    _roshandle->publishAll();
}

XBot::CommunicationHandler::~CommunicationHandler()
{
    if(_logger != nullptr) _logger->flush();
}


