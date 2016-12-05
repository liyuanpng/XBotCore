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

#include <joint_space_demo_trajectory.h>

namespace demo {
    
JointTrajectoryGenerator::JointTrajectoryGenerator(XBot::RobotInterface::Ptr robot_ptr, 
                                                   double period,
                                                   const Eigen::VectorXd& q0):
    _robot(*robot_ptr), _q0(q0), _period(period), _time(0)
{
    _robot.getJointLimits(_qmin, _qmax);
    
    Eigen::VectorXd security_margin = (_qmax - _qmin)*0.0/2;
    _qmin += security_margin;
    _qmax -= security_margin;
    
    if( !_robot.checkJointLimits(_q0) ) throw;
    
    _dq_max = (_q0 - _qmin).array().min((_qmax - _q0).array());
    
    double safety_margin = 0.15;
    _dq_max *= safety_margin;

    
}

void JointTrajectoryGenerator::getQ(double dt, Eigen::VectorXd& q)
{
    _time += dt;
    q = _q0 + _dq_max * std::sin( 2*3.1415*_time / _period );
    
    for(int i = 0; i < _robot.chain("torso").getJointNum(); i++) {
        int torso_id = _robot.getDofIndex(_robot.chain("torso").getJointName(i));
        q(torso_id) = _q0(torso_id);
    }
}



    
    
    
}