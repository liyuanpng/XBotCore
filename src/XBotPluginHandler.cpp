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

PluginHandler::PluginHandler( RobotInterface::Ptr robot, 
                       TimeProvider::Ptr time_provider,
                       XBot::SharedMemory::Ptr shared_memory,
                       Options options ) :
    _robot(robot),
    _time_provider(time_provider),
    _plugins_set_name(options.xbotcore_pluginhandler_plugin_set_name),
    _esc_utils(robot),
    _close_was_called(false),
    _shared_memory(shared_memory),
    _options(options)
{
    // plugin set mode 
    update_plugins_set_name(_options.xbotcore_pluginhandler_plugin_set_name);
    
    // Path
    _path_to_cfg = _robot->getPathToConfig();
    Logger::info() << "Plugin Handler is using config file: " << _path_to_cfg << Logger::endl();

    _pluginhandler_log = XBot::MatLogger::getLogger("/tmp/PluginHandler_log");
    Logger::info() << "With plugin set name : " << _plugins_set_name << Logger::endl();
    
    curr_plg.store(-1);
    
    _roshandle_shobj = shared_memory->getSharedObject<RosUtils::RosHandle::Ptr>("ros_handle");
    
    init_nrt_reference();
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
                
                // loading by default the XBotNRTRef
                std::string nrtref_plugin_name = "XBotNRTRef";
                it = std::find(_rtplugin_names.begin(), _rtplugin_names.end(), nrtref_plugin_name);
                if( it == _rtplugin_names.end() ) {
                    _rtplugin_names.push_back(nrtref_plugin_name);
                    _nrtref_plugin_idx = _rtplugin_names.size() - 1;
                }
                else{
                    _nrtref_plugin_idx = std::distance(_rtplugin_names.begin(), it);
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
  
        Logger::info(Logger::Severity::HIGH) << "Initializing plugin " << name << Logger::endl();
    
        bool plugin_init_success = false;
        int i=0;
        i = pluginPos[name];
       
        _plugin_state[i] == "RESTARTING";

        /* Try to init the current plugin */
        plugin_init_success = ( plugin_ptr)->init(  (XBot::Handle::Ptr) this,
                                                    name,
                                                    _plugin_custom_status[i],
                                                    _halInterface,
                                                    _model);

        /* Handle return value if init() was performed cleanly */
        if(!plugin_init_success){
            Logger::error() << "plugin " << (plugin_ptr)->name << "::init() failed. Plugin init() returned false!" << Logger::endl();           
        }
        else{
            Logger::success(Logger::Severity::HIGH) << "Plugin " << (plugin_ptr)->name << " initialized successfully!" << Logger::endl();
        }
    

        /* Handle exceptions inheriting from std::exception */
        _plugin_state[i] == "STOPPED";
        
        return plugin_init_success;
  
}

void XBot::PluginHandler::init_plugin_impl()
{

}

void XBot::PluginHandler::init_plugin_handle_except()
{

}

void XBot::PluginHandler::init_plugin_handle_stdexcept()
{

}


bool PluginHandler::init_plugins(std::shared_ptr<HALInterface> halInterface,
                                 std::shared_ptr< IXBotModel > model)
{

    if(_is_RT_plugin_handler){
        Logger::info("Waiting to receive valid RosHandle...");
        while(!_roshandle){
            _roshandle = _roshandle_shobj.get();
            usleep(1000);
        }
        Logger::info() << "Received RosHandle! " << _roshandle << Logger::endl();
    }
    else{
       _roshandle.reset( new XBot::RosUtils::RosHandle );  
    }
    
    _halInterface = halInterface;
    _joint = std::shared_ptr< IXBotJoint> (halInterface);    
    _model = model;
  
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
        // NOTE starting by default the nrtref plugin
        _plugin_state[_nrtref_plugin_idx] = "RUNNING";
    }

    bool ret = true;

    for(int i = 0; i < _rtplugin_vector.size(); i++) {

        bool plugin_init_success = false;
        _plugin_custom_status[i] = std::make_shared<PluginStatus>();
        
        if(_options.xbotcore_pluginhandler_catch_exceptions){
            try{
                plugin_init_success = initPlugin(_rtplugin_vector[i], _rtplugin_names[i]);
            }
            catch(std::exception& e){
                Logger::error() << "plugin " << _rtplugin_names[i] << "::init() failed. \n An exception was thrown: " << e.what() << Logger::endl();            
                plugin_init_success = false;
            }

            /* Handle all other exceptions */
            catch(...){
                Logger::error() << "plugin " << _rtplugin_names[i] << "::init() failed. \n An exception was thrown: " << Logger::endl();
                plugin_init_success = false;
            }
        }
        else
        {
            plugin_init_success = initPlugin(_rtplugin_vector[i], _rtplugin_names[i]);
        }
        
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
    double tic = _time_provider->get_time();
    
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

    double toc = _time_provider->get_time();
    _pluginhandler_log->add("run_xddp_exec_time", toc-tic);

}

void XBot::PluginHandler::fill_robot_state()
{
    
    double tic = _time_provider->get_time();
    
    _esc_utils.setRobotStateFromRobotInterface(_robot_state_map);
    _esc_utils.setRobotFTFromRobotInterface(_ft_state_map);
    _esc_utils.setRobotIMUFromRobotInterface(_imu_state_map);
    
    for(int id: _robot->getEnabledJointId()){
     
        double fault_value = 0;
        double temperature = 0;
        
        if(!_joint) continue;
        
        _joint->get_fault(id, fault_value);
        _joint->get_temperature(id, temperature);
         
        _robot_state_map.at(id).RobotStateRX.fault = fault_value;
        _robot_state_map.at(id).RobotStateRX.temperature = temperature;
        
    }
    
    double toc = _time_provider->get_time();
    _pluginhandler_log->add("fill_robot_state_exec_time", toc-tic);
    
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
     
    
    // log plugin handler exec time
    double plugin_handler_tic = _time_provider->get_time();

    // update robot state
    double sense_tic = _time_provider->get_time();
    _robot->sense();
    double sense_toc = _time_provider->get_time();
    _pluginhandler_log->add("robot_sense_exec_time", sense_toc - sense_tic);

    // fill robot state
    fill_robot_state();
    
    // update nrt reference 
    fill_nrt_reference();

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
                
                _pluginhandler_log->add(_rtplugin_names[i] + "_exec_time", 0.0);

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
    
    double plugin_handler_toc = _time_provider->get_time();
    _pluginhandler_log->add("plugin_handler_exec_time", plugin_handler_toc - plugin_handler_tic);
    

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
        
        /* nrtref can always start */
        
        if( plugin_idx == _nrtref_plugin_idx ){
            return true;
        }

        /* We are asked to run the communication plugin,
        allow it if and ONLY if all plugins are stopped,
        except for the logging plugin which can always run */

        if( plugin_idx == _communication_plugin_idx ){

            bool can_start = true;

            for(int i = 0; i < _plugin_state.size(); i++){
                can_start = can_start && ( _plugin_state[i] == "STOPPED" || i == _logging_plugin_idx || i == _nrtref_plugin_idx );
            }
            
            if(!can_start){
                Logger::error() << "Cannot run RT plugin 'XBotCommunicationPlugin' if other RT plugins are active. Running plugins are: \n";
                for(int i = 0; i < _plugin_state.size(); i++){
                    if(!(_plugin_state[i] == "STOPPED") && (_rtplugin_names[i] != "XBotLoggingPlugin")){
                        Logger::log() << "    " << _rtplugin_names[i] << "\n";
                    }
                }
                Logger::log() << Logger::endl();
            }

            return can_start;

        }
        else{

            /* We are asked to run a normal plugin. Allow it
            * only if the communication plugin is not running */
            
            bool can_start = _plugin_state[_communication_plugin_idx] == "STOPPED";
            
            if(!can_start){
                Logger::error() << "Cannot run a RT plugin if XBotCommunicationPlugin is switched on!" << Logger::endl();
            }

            return can_start;
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

XBot::RosUtils::RosHandle::Ptr XBot::PluginHandler::getRosHandle() const
{
    return _roshandle;
}

bool XBot::PluginHandler::getNrtPositionReference(JointIdMap& pos_id_map) const
{
    pos_id_map = _nrt_pos;
    return true;
}

bool XBot::PluginHandler::getNrtVelocityReference(JointIdMap& vel_id_map) const
{
    vel_id_map = _nrt_vel;
    return true;
}

bool XBot::PluginHandler::getNrtEffortReference(JointIdMap& eff_id_map) const
{
    eff_id_map = _nrt_eff;
    return true;
}

bool XBot::PluginHandler::getNrtImpedanceReference(JointIdMap& k_id_map, 
                                                   JointIdMap& d_id_map) const
{
    k_id_map = _nrt_imp_k;
    d_id_map = _nrt_imp_d;
    return true;
}

void XBot::PluginHandler::fill_nrt_reference()
{
    double tic = _time_provider->get_time();
    
    (_ref_map_so.at("pos_ref_map_so")).get(_nrt_pos);
    (_ref_map_so.at("vel_ref_map_so")).get(_nrt_vel);
    (_ref_map_so.at("tor_ref_map_so")).get(_nrt_eff);
    (_ref_map_so.at("k_ref_map_so")).get(_nrt_imp_k);
    (_ref_map_so.at("d_ref_map_so")).get(_nrt_imp_d);
    
    double toc = _time_provider->get_time();
    _pluginhandler_log->add("fill_nrt_reference_exec_time", toc-tic);
}

void XBot::PluginHandler::init_nrt_reference()
{
    _ref_map_so["pos_ref_map_so"] = getSharedMemory()->getSharedObject<XBot::JointIdMap>("pos_ref_map_so");
    _ref_map_so["vel_ref_map_so"] = getSharedMemory()->getSharedObject<XBot::JointIdMap>("vel_ref_map_so");
    _ref_map_so["tor_ref_map_so"] = getSharedMemory()->getSharedObject<XBot::JointIdMap>("tor_ref_map_so");
    _ref_map_so["k_ref_map_so"] = getSharedMemory()->getSharedObject<XBot::JointIdMap>("k_ref_map_so");
    _ref_map_so["d_ref_map_so"] = getSharedMemory()->getSharedObject<XBot::JointIdMap>("d_ref_map_so");
}


}
