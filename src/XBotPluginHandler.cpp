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
#include <string>

#include <unistd.h>
#include <spawn.h>
#include <sys/wait.h>

#include <XCM/PluginFactory.h>

#include <XBotInterface/RtLog.hpp>


extern char **environ;

namespace XBot {

PluginHandler::PluginHandler(RobotInterface::Ptr robot,
                             TimeProvider::Ptr time_provider,
                             const std::string& plugins_set_name) :
    _robot(robot),
    _time_provider(time_provider),
    _plugins_set_name(plugins_set_name),
    _esc_utils(robot),
    _close_was_called(false)
{
    // plugin set mode
    update_plugins_set_name(_plugins_set_name);

    // Path
    _path_to_cfg = _robot->getPathToConfig();
    Logger::info() << "Plugin Handler is using config file: " << _path_to_cfg << Logger::endl();

    _pluginhandler_log = XBot::MatLogger::getLogger("/tmp/PluginHandler_log");
    Logger::info() << "With plugin set name : " << _plugins_set_name << Logger::endl();

    curr_plg.store(-1);
}

void XBot::PluginHandler::update_plugins_set_name(const std::string& plugins_set_name)
{
    _plugins_set_name = _plugins_set_name;
     if( _plugins_set_name == "XBotRTPlugins") {
        // saving RT PluginHandler mode
        _is_RT_plugin_handler = true;
    }
    else {
        // NRT PluginHandler
        _is_RT_plugin_handler = false;
    }
}

std::vector<std::string>& PluginHandler::getPluginsName()
{

  return _rtplugin_names;

}

std::shared_ptr<XBot::XBotControlPlugin> PluginHandler::loadPlugin(const std::string& plugin_name) {


     std::shared_ptr<XBot::XBotControlPlugin> plugin_ptr = PluginFactory::getFactory("lib"+plugin_name, plugin_name);
     if(!plugin_ptr) {
      Logger::error() << "Unable to load plugin " << plugin_name << "!" << Logger::endl();
          }
       else{
        Logger::info(Logger::Severity::HIGH) << "Restarting plugin " << plugin_name << "!" << Logger::endl();
    }

    return plugin_ptr;
}

bool PluginHandler::load_plugins()
{

    std::ifstream fin(_path_to_cfg);
    if (fin.fail()) {
        Logger::error() << "in " << __func__ << "! Can NOT open config file " << _path_to_cfg << "!" << Logger::endl();
        return false;
    }

    _root_cfg = YAML::LoadFile(_path_to_cfg);


    if(!_root_cfg[_plugins_set_name]){
        Logger::error() << "in " << __func__ << "! Config file does NOT contain mandatory node " << _plugins_set_name << "!" << Logger::endl();
        return false;
    }
    else{

        // NOTE we always expect a subfield plugins inside _plugins_set_name
        if(!_root_cfg[_plugins_set_name]["plugins"]){
            Logger::error() << "in " << __func__ << "! " << _plugins_set_name << " node does NOT contain mandatory node plugins!" << Logger::endl();
        return false;
        }
        else{

            for(const auto& plugin : _root_cfg[_plugins_set_name]["plugins"]){
                _rtplugin_names.push_back(plugin.as<std::string>());
            }

            // NOTE only for RT plugins
            if( _is_RT_plugin_handler ) {

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

    }

    bool success = true;

    int pos = 0;

    for( const std::string& plugin_name : _rtplugin_names ){

         std::shared_ptr<XBot::XBotControlPlugin> plugin_ptr = PluginFactory::getFactory("lib"+plugin_name, plugin_name);
         if(!plugin_ptr) {
           success = false;
           continue;
          }

          _rtplugin_vector.push_back(plugin_ptr);

        _pluginhandler_log->createScalarVariable(plugin_name + "_exec_time");

        std::string plugin_name1 = plugin_name;
        //pluginMap[plugin_name1] = plugin_ptr;
        pluginPos [plugin_name1] = pos;
        pos = pos +1;
    }
    _time.resize(_rtplugin_vector.size());
    _last_time.resize(_rtplugin_vector.size());
    _period.resize(_rtplugin_vector.size());
    return success;
}

bool PluginHandler::initPlugin(  std::shared_ptr<XBot::XBotControlPlugin> plugin_ptr,
                                 const std::string& name)
{

        Logger::info(Logger::Severity::HIGH) << "Initializing plugin " << plugin_ptr->name << Logger::endl();

        bool plugin_init_success = false;
        int i=0;
        i = pluginPos[name];

         _plugin_state[i] == "RESTARTING";
        try{
            /* Try to init the current plugin */
            plugin_init_success = ( plugin_ptr)->init(  (XBot::Handle::Ptr) this,
                                                        name,
                                                        _plugin_custom_status[i],
                                                        _joint,
                                                        _model,
                                                        _ft,
                                                        _imu,
                                                        _hand );

            /* Handle return value if init() was performed cleanly */
            if(!plugin_init_success){
                Logger::error() << "plugin " << (plugin_ptr)->name << "::init() failed. Plugin init() returned false!" << Logger::endl();
            }
            else{
                Logger::success(Logger::Severity::HIGH) << "Plugin " << (plugin_ptr)->name << " initialized successfully!" << Logger::endl();
            }
        }

        /* Handle exceptions inheriting from std::exception */
        catch(std::exception& e){
            Logger::error() << "plugin " << (plugin_ptr)->name << "::init() failed. \n An exception was thrown: " << e.what() << Logger::endl();
            plugin_init_success = false;
        }

        /* Handle all other exceptions */
        catch(...){
            Logger::error() << "plugin " << (plugin_ptr)->name << "::init() failed. \n An exception was thrown: " << Logger::endl();
            plugin_init_success = false;
        }

        _plugin_state[i] == "STOPPED";

        return plugin_init_success;

}

bool PluginHandler::init_plugins(XBot::SharedMemory::Ptr shared_memory,
                                 std::shared_ptr< IXBotJoint> joint,
                                 std::shared_ptr< IXBotFT > ft,
                                 std::shared_ptr< IXBotIMU > imu,
                                 std::shared_ptr< IXBotHand > hand,
                                 std::shared_ptr< IXBotModel > model )
{

    _shared_memory = shared_memory;
    _joint = joint;
    _ft = ft;
    _imu = imu;
    _hand = hand;
    _model = model;

    // Save xbot_joint
    _xbot_joint = joint;

    _plugin_init_success.resize(_rtplugin_vector.size(), false);
    _plugin_switch.resize(_rtplugin_vector.size());
    _plugin_status.resize(_rtplugin_vector.size());
    _plugin_cmd.resize(_rtplugin_vector.size());
    _plugin_custom_status.resize(_rtplugin_vector.size());
    _plugin_state.resize(_rtplugin_vector.size(), "STOPPED");
    _first_loop.resize(_rtplugin_vector.size(), true);

    // NOTE xddp initialization only if we are handling RT Plugins
    if( _is_RT_plugin_handler ) {
        init_xddp();

        // NOTE starting by default the logging plugin
        _plugin_state[_logging_plugin_idx] = "RUNNING";
    }

    bool ret = true;

    for(int i = 0; i < _rtplugin_vector.size(); i++) {

        Logger::info(Logger::Severity::HIGH) << "Initializing plugin " << _rtplugin_names[i] << Logger::endl();

        bool plugin_init_success = false;
        _plugin_custom_status[i] = std::make_shared<PluginStatus>();

        plugin_init_success = initPlugin(_rtplugin_vector[i], _rtplugin_names[i]);

        // allocate concrete pub/sub classes
        if( _is_RT_plugin_handler ) {
            _plugin_switch[i] = std::make_shared<XBot::SubscriberRT<XBot::Command>>();
            _plugin_status[i] = std::make_shared<XBot::PublisherRT<XBot::Command>>();
            _plugin_cmd[i] = std::make_shared<XBot::SubscriberRT<XBot::Command>>();
            _plugin_cmd[i]->init(_rtplugin_names[i] + "_cmd");
        }
        else {
            _plugin_switch[i] = std::make_shared<XBot::NRT_ROS_Subscriber>();
            _plugin_status[i] = std::make_shared<XBot::NRT_ROS_Publisher>();
            // NOTE handling cmd for NRTPlugin in CH style
            _plugin_cmd[i] = std::make_shared<XBot::NRT_ROS_Subscriber>();
            _plugin_cmd[i]->init(_rtplugin_names[i]+"_cmd");
        }

        // initialize pub/sub
        _plugin_init_success[i] = plugin_init_success;
        _plugin_switch[i]->init(_rtplugin_names[i]+"_switch");
        _plugin_status[i]->init(_rtplugin_names[i]+"_status");
    }

    return ret;
}

bool XBot::PluginHandler::init_xddp()
{
    // Motor
    for( int id : _robot->getEnabledJointId() ) {
        XBot::PublisherRT<XBot::RobotState> pub(std::string("Motor_id_") + std::to_string(id));
        _motor_pub_map[id] = pub;
        // NOTE preallocate memory beacause of XENOMAI
        _robot_state_map[id] = XBot::RobotState();
    }

    // FT
    for( const auto& ft : _robot->getForceTorque() ) {
        int id = ft.second->getSensorId();
        XBot::PublisherRT<XBot::RobotFT::pdo_rx> pub(std::string("Ft_id_") + std::to_string(id));
        _ft_pub_map[id] = pub;
    }

    // IMU
    for( const auto& imu : _robot->getImu() ) {
        int id = imu.second->getSensorId();
        XBot::PublisherRT<XBot::RobotIMU::pdo_rx> pub(std::string("Imu_id_") + std::to_string(id));
        _imu_pub_map[id] = pub;
    }

    // HAND
    for( const auto& h : _robot->getHand() ) {
        int id = h.second->getHandId();
        XBot::PublisherRT<XBot::RobotState> pub(std::string("Motor_id_") + std::to_string(id));
        _motor_pub_map[id] = pub;
        // NOTE preallocate memory beacause of XENOMAI
        _robot_state_map[id] = XBot::RobotState();
    }
}


void XBot::PluginHandler::run_xddp()
{
    // Motor + Hand
    for( auto& pub_motor : _motor_pub_map ) {
        pub_motor.second.write(_robot_state_map.at(pub_motor.first));
    }

    // FT
    for( auto& pub_ft : _ft_pub_map ) {
        pub_ft.second.write(_ft_state_map.at(pub_ft.first));
    }

    // IMU
    for( auto& pub_imu : _imu_pub_map ) {
        pub_imu.second.write(_imu_state_map.at(pub_imu.first));
    }


}

void XBot::PluginHandler::fill_robot_state()
{
    _esc_utils.setRobotStateFromRobotInterface(_robot_state_map);
    _esc_utils.setRobotFTFromRobotInterface(_ft_state_map);
    _esc_utils.setRobotIMUFromRobotInterface(_imu_state_map);

    for(int id: _robot->getEnabledJointId()){

        double fault_value = 0;
        double temperature = 0;
        double aux = 0;

        if(!_xbot_joint) continue;

        _xbot_joint->get_fault(id, fault_value);
        _xbot_joint->get_temperature(id, temperature);
        _xbot_joint->get_aux(id, aux);

        _robot_state_map.at(id).RobotStateRX.fault = fault_value;
        _robot_state_map.at(id).RobotStateRX.temperature = aux;
        _robot_state_map.at(id).RobotStateRX.aux = aux;

    }

}

void PluginHandler::replacePlugin(const std::string& name){

    int pos = pluginPos[name];
    curr_plg.store(pos);
    if( _plugin_state[pos].compare("STOPPED") != 0) {curr_plg.store(-1); return;}
    unloadPlugin(name);
    std::shared_ptr<XBot::XBotControlPlugin> plugin_ptr = loadPlugin(name);
    initPlugin(plugin_ptr, name);
    _rtplugin_vector[pos] = plugin_ptr;
    curr_plg.store(-1);
}

void PluginHandler::run()
{

    // update robot state
    _robot->sense();

    // fill robot state
    fill_robot_state();

    // NOTE in the RT case broadcast robot state over pipes
    if( _is_RT_plugin_handler ) {
        run_xddp();
    }

    XBot::Command cmd;

    for( int i = 0; i < _rtplugin_vector.size(); i++){

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

        if( curr_plg.load() != i){
            if( _plugin_state[i] == "STOPPED" ){

                _plugin_status[i]->write(XBot::Command("STOPPED"+_plugin_custom_status[i]->getStatus()));

                if( _plugin_switch[i]->read(cmd) ){

                    /* If start command has been received, set plugin to RUNNING */
                    if( cmd.str() == "start" && plugin_can_start(i) ){
                        const auto& plugin = _rtplugin_vector[i];
                        Logger::info(Logger::Severity::HIGH) << "Starting plugin : " << (plugin)->name << Logger::endl();
                        (plugin)->on_start(_time[i]);
                        _plugin_state[i] = "RUNNING";
                    }
                }
            }
        }

        /* STATE RUNNING */

        if( curr_plg.load() != i){
            if( _plugin_state[i] == "RUNNING" ){

                const auto& plugin = _rtplugin_vector[i];
                _plugin_status[i]->write(XBot::Command("RUNNING"+_plugin_custom_status[i]->getStatus()));

                if( _plugin_switch[i]->read(cmd) ){

                    /* If stop command has been received, set plugin to STOPPED */
                    if( cmd.str() == "stop" ){
                        Logger::info(Logger::Severity::HIGH) << "Stopping plugin : " << (plugin)->name << Logger::endl();
                        (plugin)->on_stop(_time[i]);
                        _plugin_state[i] = "STOPPED";
                    }
                }

                _plugin_cmd[i]->read((plugin)->getCmd());

                double tic = _time_provider->get_time();
                (plugin)->run(_time[i], _period[i]);
                double toc = _time_provider->get_time();

                XBot::Command cm;
                (plugin)->setCmd(cm);

                _pluginhandler_log->add(_rtplugin_names[i] + "_exec_time", toc-tic);

            }
        }

    }
    _last_time = _time;

}

void PluginHandler::unloadPlugin(const std::string& port_name)
{
   int pos = pluginPos[port_name];
   _rtplugin_vector[pos].reset();
   PluginFactory::unloadLib("lib"+port_name);
}

void PluginHandler::close()
{
    if(_close_was_called) return;

    _close_was_called = true;

    int i=0;
    for( const auto& plugin : _rtplugin_vector ){
        plugin->close();
        _rtplugin_vector[i].reset();
//         PluginFactory::unloadLib("lib"+_rtplugin_names[i]);
        i++;
    }
    _pluginhandler_log->flush();
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
            Logger::error() << "in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << Logger::endl();
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
    // NOTE Policy for RT Plugins
    if( _is_RT_plugin_handler ) {

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

    // relaxed policy for NRT Plugins
    return true;
}

const std::string& XBot::PluginHandler::getPathToConfigFile() const
{
    return _path_to_cfg;
}

RobotInterface::Ptr XBot::PluginHandler::getRobotInterface() const
{
    return _robot;
}

SharedMemory::Ptr XBot::PluginHandler::getSharedMemory() const
{
    return _shared_memory;
}




}
