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

#ifndef __XCM_EXAMPLES_HOMING_EXAMPLE_H__
#define __XCM_EXAMPLES_HOMING_EXAMPLE_H__

#include <XCM/XBotControlPlugin.h>

namespace XBot {
 
    class HomingExample : public XBotControlPlugin {
      
    public:
        
        HomingExample();
        
        virtual bool close();
        
       

    protected:
        
        virtual void control_loop(double time, double period);
        
        virtual bool init_control_plugin(RobotInterface::Ptr robot);
        
    private:
        
        RobotInterface::Ptr _robot;
        
        Eigen::VectorXd _q0, _q_home;
        
        double _time;
        
    };
    
}

#endif // __XCM_EXAMPLES_HOMING_EXAMPLE_H__