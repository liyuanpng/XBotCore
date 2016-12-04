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

#ifndef __XCM_EXAMPLES_JOINT_SPACE_DEMO_TRAJ_H__
#define __XCM_EXAMPLES_JOINT_SPACE_DEMO_TRAJ_H__

#include <XBotInterface/RobotInterface.h>

namespace demo {
 
    class JointTrajectoryGenerator {
    
    public:
        
        JointTrajectoryGenerator(XBot::RobotInterface::Ptr robot_ptr, 
                                 double period,
                                 const Eigen::VectorXd& q0);
        
        void getQ(double dt, Eigen::VectorXd& q);
        
    protected:
        
    private:
        
        
        XBot::RobotInterface& _robot;
        Eigen::VectorXd _qmin, _qmax, _q0;
        Eigen::VectorXd _dq_max;
        mutable Eigen::VectorXd _alpha;
        double _time;
        
        double _period;
        
        
    };

    
}

#endif