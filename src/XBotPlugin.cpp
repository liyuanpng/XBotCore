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

#include <XBotCore-interfaces/XBotPlugin.h>

namespace XBot {
    
XBotPlugin::XBotPlugin()
{

}

XBotPlugin::~XBotPlugin()
{

}

std::shared_ptr< HALInterface > XBotPlugin::get_xbotcore_halInterface()
{
    return _halInterface;
}

std::shared_ptr< XBot::IXBotJoint > XBotPlugin::get_xbotcore_joint()
{
    return _joint;
}

std::shared_ptr< XBot::IXBotModel > XBotPlugin::get_xbotcore_model()
{
    return _model;
}

std::shared_ptr< XBot::IXBotFT > XBotPlugin::get_xbotcore_ft()
{
    return _ft;
}

std::shared_ptr< IXBotIMU > XBotPlugin::get_xbotcore_imu()
{
    return _imu;
}

std::shared_ptr< XBot::IXBotHand > XBot::XBotPlugin::get_xbotcore_hand()
{
    return _hand;
}

void XBotPlugin::set_xbotcore_halInterface(std::shared_ptr< HALInterface > halInterface)
{
    _halInterface = halInterface;
}

void XBotPlugin::set_xbotcore_joint(std::shared_ptr< XBot::IXBotJoint > joint)
{
    _joint = joint;
}

void XBotPlugin::set_xbotcore_model(std::shared_ptr< XBot::IXBotModel > model)
{
    _model = model;
}

void XBotPlugin::set_xbotcore_ft(std::shared_ptr< XBot::IXBotFT > ft)
{
    _ft = ft;
}

void XBotPlugin::set_xbotcore_imu(std::shared_ptr< IXBotIMU > imu)
{
    _imu = imu;
}

void XBot::XBotPlugin::set_xbotcore_hand ( std::shared_ptr< XBot::IXBotHand > hand )
{
    _hand = hand;
}


    
}