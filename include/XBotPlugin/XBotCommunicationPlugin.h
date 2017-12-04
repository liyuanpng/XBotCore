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

#ifndef __X_BOT_COMMUNICATION_PLUGIN_H__
#define __X_BOT_COMMUNICATION_PLUGIN_H__

#include <XCM/XBotControlPlugin.h>

#include <XCM/XBotThread.h>
#include <XCM/XBotCommunicationHandler.h>

#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <XBotInterface/Utils.h>

#include <memory>

namespace XBot
{
    class XBotCommunicationPlugin;
}

/**
 * @brief XBotCore RT plugin that communicates wit the non-RT enviroment using XDDP pipes
 *
 */
class XBot::XBotCommunicationPlugin : public XBot::XBotControlPlugin
{
public:
    XBotCommunicationPlugin();

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void on_stop(double time);

    virtual bool close();

    virtual ~XBotCommunicationPlugin();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::SharedObject<XBot::JointIdMap> _pos_ref_map, _vel_ref_map, _tor_ref_map, _k_ref_map, _d_ref_map;
    Eigen::VectorXd _q0, _qref, _k0, _kref, _d0, _dref, _qdot0, _qdotref;

    double _start_time;

    XBot::RobotInterface::Ptr _robot;

    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _filter_q, _filter_k, _filter_d, _filter_qdot;

    bool _filter_enabled;

    Thread_hook::Ptr _ch;

};

#endif //__X_BOT_COMMUNICATION_PLUGIN_H__
