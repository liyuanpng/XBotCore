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

#ifndef __XBOT_HANDLE_H__
#define __XBOT_HANDLE_H__

#include <XBotCore-interfaces/XBotSharedMemory.h>
#include <XBotInterface/RobotInterface.h>

#include <XBotCore-interfaces/XBotRosUtils.h>

namespace XBot {

    class Handle {
        
    public:
        
        typedef Handle* Ptr;
        
        virtual const std::string&  getPathToConfigFile() const = 0;
        virtual SharedMemory::Ptr   getSharedMemory()     const = 0;
        virtual RobotInterface::Ptr getRobotInterface()   const = 0;
        virtual RosUtils::RosHandle::Ptr getRosHandle()   const = 0;
        virtual bool getNrtPositionReference(XBot::JointIdMap& pos_id_map) const = 0;
        virtual bool getNrtVelocityReference(XBot::JointIdMap& vel_id_map) const = 0;
        virtual bool getNrtEffortReference(XBot::JointIdMap& eff_id_map) const = 0;
        virtual bool getNrtImpedanceReference(XBot::JointIdMap& k_id_map, XBot::JointIdMap& d_id_map) const = 0;
        
        
    };



}

#endif //__XBOT_HANDLE_H__

