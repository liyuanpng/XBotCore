/*
 * Copyright (C) 2016 IIT-ADVR
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

#ifndef __XBOTCONTROLMODULE_XBOTCONTROLPLUGIN_H__
#define __XBOTCONTROLMODULE_XBOTCONTROLPLUGIN_H__

#include <XBotInterface/RobotInterface.h>
#include <XBotCore-interfaces/XBotPlugin.h>

#include <XBotCore-interfaces/XDomainCommunication.h>
#include <XCM/XBotPluginStatus.h>

#include <XBotCore-interfaces/XBotHandle.h>

#define REGISTER_XBOT_PLUGIN_(plugin_name) \
extern "C" XBot::XBotControlPlugin* create_instance() \
{ \
  return new  plugin_name(); \
}\
\
extern "C" void destroy_instance( XBot::XBotControlPlugin* instance ) \
{ \
  delete instance; \
}\

//for reverse compatibility
#define REGISTER_XBOT_PLUGIN(others,plugin_name) \
extern "C" XBot::XBotControlPlugin* create_instance() \
{ \
  return new  plugin_name(); \
}\
\
extern "C" void destroy_instance( XBot::XBotControlPlugin* instance ) \
{ \
  delete instance; \
}\

namespace XBot {

class XBotControlPlugin : public XBotPlugin {

public:

    XBotControlPlugin();

    virtual ~XBotControlPlugin();

    virtual bool init(XBot::Handle::Ptr handle,
                      std::string name,
                      std::shared_ptr< PluginStatus > cstatus,                      
                      std::shared_ptr<HALInterface> halInterface,
                      std::shared_ptr< XBot::IXBotModel > model) final;

    virtual bool init_control_plugin(XBot::Handle::Ptr handle) = 0;

    virtual void on_start(double time) {};

    virtual void on_stop(double time) {};

    virtual void run(double time, double period) final;
    
    XBot::Command& getCmd();
    
    void setCmd( XBot::Command& cmd);
    

protected:

    virtual void control_loop(double time, double period) = 0;

    XBot::Command current_command;
    
    std::shared_ptr<PluginStatus> _custom_status;
    
private:


};

}

#endif
