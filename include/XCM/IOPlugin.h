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

#ifndef __XCM_IOPLUGIN_H__
#define __XCM_IOPLUGIN_H__

#include <string>
#include <XBotInterface/XBotInterface.h>

#define REGISTER_XBOT_IO_PLUGIN(others,plugin_name) \
extern "C" XBot::IOPlugin* create_instance() \
{ \
  return new  plugin_name(); \
}\
\
extern "C" void destroy_instance( XBot::IOPlugin* instance ) \
{ \
  delete instance; \
}\


#define REGISTER_XBOT_IO_PLUGIN_(plugin_name) \
extern "C" XBot::IOPlugin* create_instance() \
{ \
  return new  plugin_name(); \
}\
\
extern "C" void destroy_instance( XBot::IOPlugin* instance ) \
{ \
  delete instance; \
}\

namespace XBot {

class IOPlugin {

public:

    virtual bool init(std::string path_to_config_file) = 0;

    virtual void run() = 0;

    virtual void close() = 0;

};
/*
class IOPluginLoader {

public:

    bool load(std::string plugin_name);
    IOPlugin* getPtr();

private:

    static bool computeAbsolutePath (const std::string& input_path,
                                     const std::string& middle_path,
                                     std::string& absolute_path);

    std::shared_ptr<shlibpp::SharedLibraryClassFactory<IOPlugin>> _ioplugin_factory;
    std::shared_ptr<shlibpp::SharedLibraryClass<IOPlugin>> _ioplugin_class;

    bool _load_success;

};*/



}



#endif