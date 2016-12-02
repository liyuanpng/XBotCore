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

#ifndef __X_BOT_PLUGIN_HANDLER_H__
#define __X_BOT_PLUGIN_HANDLER_H__

#include <XBotCore/XBotCore.h>
#include <XBotCore/XBotPlugin.h>
#include <SharedLibraryClassFactory.h>
#include <SharedLibraryClass.h>

namespace XBot
{
    class XBotPluginHandler;
}


/**
 * @brief XBotCore Plugin Handler: it executes sequentially a set of plugins.
 * 
 */
class XBot::XBotPluginHandler : public XBot::XBotCore
{
public:
    
    XBotPluginHandler(const char * config_yaml);
    virtual ~XBotPluginHandler();

    virtual bool plugin_handler_init(void) final;
    virtual bool plugin_handler_loop(void) final;
    virtual bool plugin_handler_close(void) final;

protected:
            
    

private: 

    bool load_plugins();
    
    static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& middle_path,
                                       std::string& absolute_path ); // TBD do it with UTILS
    
    std::string _path_to_config_file;
    
    std::vector<double> _last_time, _time, _period, _elapsed_time;
     bool _first_loop;
     
    // Dynamic loading related variables
    std::vector<std::shared_ptr<shlibpp::SharedLibraryClassFactory<XBot::XBotPlugin>>> _rtplugin_factory;
    std::vector<std::string> _rtplugin_names;
    std::vector<std::shared_ptr<shlibpp::SharedLibraryClass<XBot::XBotPlugin>>> _rtplugin_vector;


};

#endif //__X_BOT_PLUGIN_HANDLER_H__
