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


#ifndef __XCM_XBOT_PLUGIN_HANDLER_H__
#define __XCM_XBOT_PLUGIN_HANDLER_H__

#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/All.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <SharedLibraryClassFactory.h>
#include <SharedLibraryClass.h>

namespace XBot {

    class PluginHandler {

    public:

        PluginHandler(RobotInterface::Ptr robot, std::string path_to_cfg);

        bool load_plugins();

        bool init_plugins();

        void run(double time);

        void close();

        ~PluginHandler();

    protected:

    private:


        static bool computeAbsolutePath ( const std::string& input_path,
                                        const std::string& midlle_path,
                                        std::string& absolute_path ); // TBD do it with UTILS

        YAML::Node _root_cfg;

        bool _close_was_called;

        // Dynamic loading related variables
        std::vector<std::shared_ptr<shlibpp::SharedLibraryClassFactory<XBot::XBotControlPlugin>>> _rtplugin_factory;
        std::vector<std::string> _rtplugin_names;
        std::vector<std::shared_ptr<shlibpp::SharedLibraryClass<XBot::XBotControlPlugin>>> _rtplugin_vector;
        std::vector<bool> _plugin_init_success;
        std::vector<XBot::SubscriberRT<XBot::Command>> _plugin_command;
        std::vector<std::string> _plugin_state;
        std::vector<bool> _first_loop;

        std::vector<double> _last_time, _time, _period, _elapsed_time;

        RobotInterface::Ptr _robot;
        std::string _path_to_cfg;
    };
}
#endif