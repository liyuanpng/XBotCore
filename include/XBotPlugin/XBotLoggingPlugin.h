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

#ifndef __X_BOT_LOGGING_PLUGIN_H__
#define __X_BOT_LOGGING_PLUGIN_H__

#include <XCM/XBotControlPlugin.h>

#include <memory>

namespace XBot
{
    class XBotLoggingPlugin;
}

/**
 * @brief XBotCore RT plugin that communicates wit the non-RT enviroment using XDDP pipes
 *
 */
class XBot::XBotLoggingPlugin : public XBot::XBotControlPlugin
{
public:
    XBotLoggingPlugin();

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void on_stop(double time);

    virtual bool close();

    virtual ~XBotLoggingPlugin();

protected:

    virtual void control_loop(double time, double period);

private:

    bool init_xddp();

    void run_xddp();

    RobotInterface::Ptr _robot;

    std::vector<double> _faults;
    std::vector<double> _aux;

    XBot::MatLogger::Ptr _logger;

};

#endif //__X_BOT_COMMUNICATION_PLUGIN_H__
