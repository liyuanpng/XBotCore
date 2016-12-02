/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#include <XBotCore/XBotPluginHandler.h>

#include <XBotPlugin/XBotCommunicationPlugin.h>

XBot::XBotPluginHandler::XBotPluginHandler(const char* config_yaml) : 
    XBotCore(config_yaml), 
    _path_to_config_file(config_yaml)
{
    // TBD read the plugins to load from YAML
}

bool XBot::XBotPluginHandler::load_plugins() {
    
    std::ifstream fin(_path_to_config_file);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << _path_to_config_file << "!" << std::endl;
        return false;
    }
    

    YAML::Node root_cfg = YAML::LoadFile(_path_to_config_file);
    
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
        
        auto factory_ptr = std::make_shared<shlibpp::SharedLibraryClassFactory<XBot::XBotPlugin>>(path_to_so.c_str(), factory_name.c_str());
        
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
        
        auto plugin_ptr = std::make_shared<shlibpp::SharedLibraryClass<XBot::XBotPlugin>>(*factory_ptr);
        
        _rtplugin_vector.push_back(plugin_ptr);

    }

    return success;

}


bool XBot::XBotPluginHandler::plugin_handler_init(void)
{
    // load the plugins
    if(!load_plugins()) {
        DPRINTF("ERROR: load_plugins() failed.");
        return false;
    }
    
    XBot::XBotCoreModel model = get_robot_model();

    std::shared_ptr<XBot::IXBotModel> actual_model = std::make_shared<XBot::XBotCoreModel>(model);
    std::shared_ptr<XBot::IXBotJoint> actual_joint(this, [](XBot::IXBotJoint* ptr){return;});
    std::shared_ptr<XBot::IXBotChain> actual_chain(this, [](XBot::IXBotChain* ptr){return;});
    std::shared_ptr<XBot::IXBotRobot> actual_robot(this, [](XBot::IXBotRobot* ptr){return;});
    std::shared_ptr<XBot::IXBotFT> actual_ft(this, [](XBot::IXBotFT* ptr){return;});
    
    // iterate over the plugins and call the init()
    int plugins_num = _rtplugin_vector.size();
    bool ret = true;
    for(int i = 0; i < plugins_num; i++) {
        if(!(*_rtplugin_vector[i])->init(_path_to_config_file, 
                                         _rtplugin_names[i],
                                         actual_joint,
                                         actual_model, 
                                         actual_chain,
                                         actual_robot,
                                         actual_ft)) 
        {
            DPRINTF("ERROR: plugin %s - init() failed\n", (*_rtplugin_vector[i])->name.c_str());
            ret = false;
        }
    }
    return ret;
}


bool XBot::XBotPluginHandler::plugin_handler_loop(void)
{
    std::vector<float> plugin_execution_time(_rtplugin_vector.size()); // TBD circular array and write to file in the plugin_handler_close
    for(int i = 0; i < _rtplugin_vector.size(); i++) {
        float plugin_start_time = (iit::ecat::get_time_ns() / 10e3); //microsec
        (*_rtplugin_vector[i])->run(0, -1); // TBD actual time
        plugin_execution_time[i] = (iit::ecat::get_time_ns() / 10e3) - plugin_start_time; //microsec
//         DPRINTF("Plugin %d - %s : execution_time = %f microsec\n", i, plugins[i]->name.c_str(), plugin_execution_time[i]);
    }
    return true;
}

bool XBot::XBotPluginHandler::plugin_handler_close(void)
{
    bool ret = true;
    for(int i = 0; i < _rtplugin_vector.size(); i++) {
        if(!(*_rtplugin_vector[i])->close()) {
            DPRINTF("ERROR: plugin %s - close() failed\n", (*_rtplugin_vector[i])->name.c_str());
            ret = false;
        }
    }
    return ret;
}

XBot::XBotPluginHandler::~XBotPluginHandler()
{
    printf("~XBotPluginHandler()\n");
}

bool XBot::XBotPluginHandler::computeAbsolutePath(const std::string& input_path, 
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
