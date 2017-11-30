/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore, Giuseppe Rigano
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it, giuseppe.rigano@iit.it
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

#ifndef __XBOT_TRANSMISSION_H__
#define __XBOT_TRANSMISSION_H__

#include <XBotInterface/XBotInterface.h>
#include <XBotCore-interfaces/All.h>

namespace XBot
{

    /**
     * @brief Interface for XBot Transmission
     * 
     */
    class Transmission
    {

    public:
        typedef std::shared_ptr<XBot::Transmission> Ptr;
        
        /**
        * @brief 
        * 
        * @return bool
        */
        virtual bool init( const std::string &path_to_cfg ) = 0;
        
        /**
        * @brief 
        * 
        * @return bool
        */
        virtual bool motorToJoint(  const std::shared_ptr<XBot::IXBotJoint> motor,
                                    XBot::JointIdMap& link_position,
                                    XBot::JointIdMap& motor_position,
                                    XBot::JointIdMap& link_velocity,
                                    XBot::JointIdMap& motor_velocity,
                                    XBot::JointIdMap& torque,
                                    XBot::JointIdMap& temperature,
                                    XBot::JointIdMap& stiffness,
                                    XBot::JointIdMap& damping ) = 0;    
        
        /**
        * @brief 
        * 
        * @return bool
        */
        virtual bool jointToMotor(  const XBot::JointIdMap& position_reference,
                                    const XBot::JointIdMap& velocity_reference,
                                    const XBot::JointIdMap& torque_reference,
                                    const XBot::JointIdMap& stiffness,
                                    const XBot::JointIdMap& damping,
                                    std::shared_ptr<XBot::IXBotJoint> motor ) = 0;
        /**
        * @brief 
        * 
        * @return bool
        */                           
        virtual bool close() { return true; };                                    

    };

}
#endif //__XBOT_TRANSMISSION_H__
