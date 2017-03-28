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

#include <XCM/XBotPluginHandler.h>

#include <XBotCore-interfaces/XBotPipes.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <spawn.h>
#include <sys/wait.h>

extern char **environ;

namespace XBot {

PluginHandler::PluginHandler(RobotInterface::Ptr robot,  TimeProvider::Ptr time_provider):
    _robot(robot),
    _time_provider(time_provider),
    _esc_utils(robot),
    _close_was_called(false)
{
    _path_to_cfg = _robot->getPathToConfig();
}


bool PluginHandler::load_plugins()
{

    _root_cfg = YAML::LoadFile(_path_to_cfg);


    if(!_root_cfg["XBotRTPlugins"]){
        std::cout << "ERROR in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << std::endl;
        return false;
    }
    else{

        if(!_root_cfg["XBotRTPlugins"]["plugins"]){
            std::cout << "ERROR in " << __func__ << "! XBotRTPlugins node does NOT contain mandatory node plugins!" << std::endl;
        return false;
        }
        else{

            for(const auto& plugin : _root_cfg["XBotRTPlugins"]["plugins"]){
                _rtplugin_names.push_back(plugin.as<std::string>());
            }

            // loading by default the XBotCommunicationPlugin
            std::string communication_plugin_name = "XBotCommunicationPlugin";
            auto it = std::find(_rtplugin_names.begin(), _rtplugin_names.end(), communication_plugin_name);
            if( it == _rtplugin_names.end() ) {
                _rtplugin_names.push_back(communication_plugin_name);
                _communication_plugin_idx = _rtplugin_names.size() - 1;
            }
            else{
                _communication_plugin_idx = std::distance(_rtplugin_names.begin(), it);
            }
            
            // loading by default the XBotLoggingPlugin
            std::string logging_plugin_name = "XBotLoggingPlugin";
            it = std::find(_rtplugin_names.begin(), _rtplugin_names.end(), logging_plugin_name);
            if( it == _rtplugin_names.end() ) {
                _rtplugin_names.push_back(logging_plugin_name);
                _logging_plugin_idx = _rtplugin_names.size() - 1;
            }
            else{
                _logging_plugin_idx = std::distance(_rtplugin_names.begin(), it);
            }
        }

    }

    bool success = true;

    for( const std::string& plugin_name : _rtplugin_names ){

        std::string path_to_so;
        computeAbsolutePath(plugin_name, "/build/install/lib/lib", path_to_so);
        path_to_so += std::string(".so");

        std::string factory_name = plugin_name + std::string("_factory");

        auto factory_ptr = std::make_shared<shlibpp::SharedLibraryClassFactory<XBot::XBotControlPlugin>>(path_to_so.c_str(), factory_name.c_str());

        if (!factory_ptr->isValid()) {
            // NOTE print to celebrate the wizard
            printf("error (%s) : %s\n", shlibpp::Vocab::decode(factory_ptr->getStatus()).c_str(),
                factory_ptr->getLastNativeError().c_str());
            std::cout << "Unable to load plugin " << plugin_name << "!" << std::endl;
            success = false;
            continue;
        }
        else{
            std::cout << "Found plugin " << plugin_name << "!" << std::endl;
        }

        _rtplugin_factory.push_back(factory_ptr);

        auto plugin_ptr = std::make_shared<shlibpp::SharedLibraryClass<XBot::XBotControlPlugin>>(*factory_ptr);

        _rtplugin_vector.push_back(plugin_ptr);

    }

    _time.resize(_rtplugin_vector.size());
    _last_time.resize(_rtplugin_vector.size());
    _period.resize(_rtplugin_vector.size());

    return success;
}

bool PluginHandler::init_plugins(std::shared_ptr< IXBotJoint> joint,
                                 std::shared_ptr< IXBotFT > ft,
                                 std::shared_ptr< IXBotModel > model)
{
    init_xddp();

    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();
    _plugin_init_success.resize(_rtplugin_vector.size(), false);
    _plugin_command.resize(_rtplugin_vector.size());
    _plugin_state.resize(_rtplugin_vector.size(), "STOPPED");
    _first_loop.resize(_rtplugin_vector.size(), true);

    bool ret = true;
    
    // NOTE starting by default the logging plugin
    _plugin_state[_logging_plugin_idx] = "RUNNING";

    for(int i = 0; i < _rtplugin_vector.size(); i++) {
        if(!(*_rtplugin_vector[i])->init( _path_to_cfg,
                                          _rtplugin_names[i],
                                          shared_memory,
                                          joint,
                                          model,
                                          ft)
             )
        {
            std::cout << "ERROR: plugin " << (*_rtplugin_vector[i])->name << " - init() failed" << std::endl;
            ret = false;
        }

        std::cout << "Plugin " << (*_rtplugin_vector[i])->name << " initialized successfully!" << std::endl;
        _plugin_init_success[i] = true;
        _plugin_command[i].init(_rtplugin_names[i]+"_switch");
    }

    return ret;
}

bool XBot::PluginHandler::init_xddp()
{
    for( int id : _robot->getEnabledJointId() ) {
        XBot::PublisherRT<XBot::RobotState> pub(std::string("Motor_id_") + std::to_string(id));
        _pub_map[id] = pub;
    }
}


void XBot::PluginHandler::run_xddp()
{
    for( auto& pub : _pub_map ) {
        pub.second.write(_robot_state_map.at(pub.first));
    }
}

void XBot::PluginHandler::fill_robot_state()
{
    _esc_utils.setRobotStateFromRobotInterface(_robot_state_map);
}


void PluginHandler::run()
{

    // update robot state
    _robot->sense();

    // fill robot state
    fill_robot_state();

    // broadcast robot state over pipes
    run_xddp();

    XBot::Command cmd;

    for( int i = 0; i < _rtplugin_vector.size(); i++){

        const auto& plugin = _rtplugin_vector[i];

        _time[i] = _time_provider->get_time();

        if(_first_loop[i]){
            _period[i] = 0;
            _first_loop[i] = false;
        }
        else{
            _period[i] = _time[i] - _last_time[i];
        }

        /* If init was unsuccessful, do not run */
        if(!_plugin_init_success[i]) continue;

        /* STATE STOPPED */

        if( _plugin_state[i] == "STOPPED" ){

            if( _plugin_command[i].read(cmd) ){

                /* If start command has been received, set plugin to RUNNING */
                if( cmd.str() == "start" && plugin_can_start(i) ){
                    std::cout << "Starting plugin : " << (*plugin)->name << std::endl;
                    (*plugin)->on_start(_time[i]);
                    _plugin_state[i] = "RUNNING";
                }
            }
        }

        /* STATE RUNNING */

        if( _plugin_state[i] == "RUNNING" ){

            if( _plugin_command[i].read(cmd) ){

                /* If stop command has been received, set plugin to STOPPED */
                if( cmd.str() == "stop" ){
                    std::cout << "Stopping plugin : " << (*plugin)->name << std::endl;
                    (*plugin)->on_stop(_time[i]);
                    _plugin_state[i] = "STOPPED";
                }
            }

            (*plugin)->run(_time[i], _period[i]);

        }

    }

    _last_time = _time;

}

void PluginHandler::close()
{
    if(_close_was_called) return;

    _close_was_called = true;

    for( const auto& plugin : _rtplugin_vector ){
        (*plugin)->close();
    }
}

bool PluginHandler::computeAbsolutePath(const std::string& input_path,
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
            XBot::ConsoleLogger::getLogger()->error() << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << XBot::ConsoleLogger::getLogger()->endl();
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}

PluginHandler::~PluginHandler()
{
    close();
}

bool PluginHandler::plugin_can_start(int plugin_idx)
{
    /* Logging plugin can always start */
    
    if( plugin_idx == _logging_plugin_idx ){
        return true;
    }

    /* We are asked to run the communication plugin,
    allow it if and ONLY if all plugins are stopped,
    except for the logging plugin which can always run */
    
    if( plugin_idx == _communication_plugin_idx ){

        bool can_start = true;

        for(int i = 0; i < _plugin_state.size(); i++){
            can_start = can_start && ( _plugin_state[i] == "STOPPED" || i == _logging_plugin_idx );
        }

        return can_start;

    }
    else{

        /* We are asked to run a normal plugin. Allow it
         * only if the communication plugin is not running */

        return _plugin_state[_communication_plugin_idx] == "STOPPED";
    }

    return false;
}



}
