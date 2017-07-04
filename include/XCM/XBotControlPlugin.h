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

#define REGISTER_XBOT_PLUGIN(plugin_name, scoped_class_name) SHLIBPP_DEFINE_SHARED_SUBCLASS(plugin_name ## _factory, scoped_class_name, XBot::XBotControlPlugin);


namespace XBot {

class XBotControlPlugin : public XBotPlugin {

public:

    XBotControlPlugin();

    virtual ~XBotControlPlugin();

    virtual bool init(std::string path_to_config_file,
                      std::string name,
                      XBot::SharedMemory::Ptr shared_memory,
                      std::shared_ptr< XBot::IXBotJoint > joint,
                      std::shared_ptr< XBot::IXBotModel > model,
                      std::shared_ptr< XBot::IXBotFT > ft,
                      std::shared_ptr< XBot::IXBotIMU > imu,
                      std::shared_ptr<XBot::IXBotHand> hand ) final;

    virtual bool init_control_plugin(std::string path_to_config_file,
                                     XBot::SharedMemory::Ptr shared_memory,
                                     RobotInterface::Ptr robot) = 0;

    virtual void on_start(double time) {};

    virtual void on_stop(double time) {};

    virtual void run(double time, double period) final;

protected:

    virtual void control_loop(double time, double period) = 0;

    XBot::SubscriberRT<XBot::Command> command;
    XBot::Command current_command;

private:


};

}

#endif
