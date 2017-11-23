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

#ifndef __X_BOT_NRT_REF_H__
#define __X_BOT_NRT_REF_H__

#include <XCM/XBotControlPlugin.h>

#include <XCM/XBotThread.h>
#include <XCM/XBotCommunicationHandler.h>

#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <XBotInterface/Utils.h>

#include <memory>

namespace XBot
{
    class XBotNRTRef;
}

/**
 * @brief XBotCore RT plugin that enables the NRT world to send reference to the RT layer using the shared memory
 *
 */
class XBot::XBotNRTRef : public XBot::XBotControlPlugin
{
public:
    
    XBotNRTRef();

    virtual bool init_control_plugin( XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void on_stop(double time);

    virtual bool close();

    virtual ~XBotNRTRef();

protected:

    virtual void control_loop(double time, double period);

private:
    
    XBot::RobotInterface::Ptr _robot;
    
    XBot::JointIdMap _pos_ref_map, _vel_ref_map, _tor_ref_map, _k_ref_map, _d_ref_map;
    
    std::map<int, XBot::SubscriberRT<XBot::RobotState::pdo_tx>> _sub_map;
    XBot::RobotState::pdo_tx _pdo_tx;
    
    std::map <int, XBot::Hand::Ptr> _hand_map;
    
    std::map<std::string, XBot::SharedObject<XBot::JointIdMap>> _ref_map_so;

};

#endif //__X_BOT_NRT_REF_H__
