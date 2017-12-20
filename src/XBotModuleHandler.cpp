#include <XCM/XBotModuleHandler.h>

#ifdef USE_ROS_COMMUNICATION_INTERFACE
#include <XCM/CommunicationInterfaceROS.h>
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
#include <XCM/XBotYARP/CommunicationInterfaceYARP.h>
#endif

namespace XBot {

ModuleHandler::ModuleHandler(std::string path_to_config_file,
                             RobotInterface::Ptr robot,
                             XBotControlModule::Ptr control_module,
                             TimeProvider::Ptr time_provider):
    _path_to_config_file(path_to_config_file),
    _robot(robot),
    _time_provider(time_provider),
    _module(control_module),
    _first_loop(true),
    _time(0), _last_time(0), _period(0),
    _state("STOPPED"),
    _close_was_called(false)
{
    /* Get a vector of communication interfaces to/from NRT frameworks like ROS, YARP, ... */

#ifdef USE_ROS_COMMUNICATION_INTERFACE
    _communication_interfaces.push_back( std::make_shared<XBot::CommunicationInterfaceROS>(robot) );
#endif

#ifdef USE_YARP_COMMUNICATION_INTERFACE
    _communication_interfaces.push_back( std::make_shared<XBot::CommunicationInterfaceYARP>(robot) );
#endif

    for(auto& comm_ifc : _communication_interfaces){
        comm_ifc->advertiseSwitch("xbot_control_module_" + control_module->get_name() + "_switch");
    }

}


bool ModuleHandler::init()
{
    return _module->init_control_module(_path_to_config_file, _robot);
}

void ModuleHandler::run()
{

    std::string command;
    bool command_received = false;
    for(auto& comm_ifc : _communication_interfaces){
        command_received = command_received ||
            comm_ifc->receiveFromSwitch("xbot_control_module_" + _module->get_name() + "_switch", command);
    }

    _time = _time_provider->get_time();

    if(_first_loop){
        _period = 0;
        _first_loop = false;
    }
    else{
        _period = _time - _last_time;
    }


    /* STATE STOPPED */

    if( _state == "STOPPED" ){

        if( command_received ){

            /* If start command has been received, set plugin to RUNNING */
            if( command == "start" ){
                _module->on_start(_time);
                _state = "RUNNING";
            }
        }
    }

    /* STATE RUNNING */

    if( _state == "RUNNING" ){

        if( command_received ){

            /* If stop command has been received, set plugin to STOPPED */
            if( command == "stop" ){
                _module->on_stop(_time);
               _state = "STOPPED";
            }
        }

        _module->run(_time, _period);

    }



    _last_time = _time;
}

void ModuleHandler::close()
{
    if(_close_was_called) return;

    _close_was_called = true;
    _module->close();
}

ModuleHandler::~ModuleHandler()
{
    close();
}




}