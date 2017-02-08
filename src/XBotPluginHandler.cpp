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

namespace XBot {

PluginHandler::PluginHandler(RobotInterface::Ptr robot, std::string path_to_cfg):
    _robot(robot),
    _path_to_cfg(path_to_cfg),
    _close_was_called(false)
{

}


bool PluginHandler::load_plugins()
{

    _root_cfg = YAML::LoadFile(_path_to_cfg);


    if(!_root_cfg["XBotRTPlugins"]){
        std::cerr << "ERROR in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << std::endl;
        return false;
    }
    else{

        if(!_root_cfg["XBotRTPlugins"]["plugins"]){
            std::cerr << "ERROR in " << __func__ << "!XBotRTPlugins node does NOT contain mandatory node plugins!" << std::endl;
        return false;
        }
        else{

            for(const auto& plugin : _root_cfg["XBotRTPlugins"]["plugins"]){
                _rtplugin_names.push_back(plugin.as<std::string>());
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
            std::cerr << "Unable to load plugin " << plugin_name << "!" << std::endl;
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

    _first_loop = true;
    _time.resize(_rtplugin_vector.size());
    _last_time.resize(_rtplugin_vector.size());
    _period.resize(_rtplugin_vector.size());

    return success;
}

bool PluginHandler::init_plugins()
{
    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();

    bool ret = true;
    for(int i = 0; i < _rtplugin_vector.size(); i++) {
        if(!(*_rtplugin_vector[i])->init_control_plugin( _path_to_cfg,
                                                         shared_memory,
                                                         _robot)
             )
        {
            printf("ERROR: plugin %s - init() failed\n", (*_rtplugin_vector[i])->name.c_str());
            ret = false;
        }
    }
    return ret;
}


void PluginHandler::run(double time)
{
    for( int i = 0; i < _rtplugin_vector.size(); i++){

        const auto& plugin = _rtplugin_vector[i];

        _time[i] = time;

        if(_first_loop){
            _period[i] = 0;
        }
        else{
            _period[i] = _time[i] - _last_time[i];
        }

        (*plugin)->run(_time[i], _period[i]);

    }

    _last_time = _time;
    _first_loop = false;
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
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
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



}
