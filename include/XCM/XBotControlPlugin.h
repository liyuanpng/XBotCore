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

namespace XBot {
 
    class XBotControlPlugin : XBotPlugin {
      
    public:
        
        XBotControlPlugin();
        
        virtual ~XBotControlPlugin();
        
        virtual bool init(std::string path_to_config_file,
                          std::string name, 
                          std::shared_ptr<XBot::IXBotJoint> joint,
                          std::shared_ptr< IXBotModel > model, 
                          std::shared_ptr< IXBotChain > chain, 
                          std::shared_ptr< IXBotRobot > robot, 
                          std::shared_ptr< IXBotFT > ft) final;
                          
        virtual bool init_control_plugin(std::string path_to_config_file, RobotInterface::Ptr robot) = 0;
                          
        virtual void run(double time, double period) final;

    protected:

        virtual void control_loop(double time, double period) = 0;
        
        double get_first_loop_time() const;
        
    private:
        
        bool _initial_time_set;
        double _initial_time;
        
        
        
    };
    
}

#endif