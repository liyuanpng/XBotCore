/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
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

#ifndef __XCM_EXAMPLES_TWO_PRIORITY_IK_PLUGIN_H__
#define __XCM_EXAMPLES_TWO_PRIORITY_IK_PLUGIN_H__

#include <twoprioritiy_rt_ik.h>
#include <joint_space_demo_trajectory.h>
#include <XCM/XBotControlPlugin.h>

namespace demo {

    class TwoPriorityIkPlugin : public XBot::XBotControlPlugin {
      
    public:
        
        virtual bool init_control_plugin(std::string path_to_config_file, XBot::RobotInterface::Ptr robot);
        virtual void control_loop(double time, double period);
        virtual bool close();
            
    protected:
        
    private:
        
        std::shared_ptr<TwoPriorityRtIk> _ik;
        std::shared_ptr<JointTrajectoryGenerator> _generator;
        XBot::RobotInterface::Ptr _robot;
        XBot::ModelInterface::Ptr _model, _model_traj;
        
        Eigen::VectorXd _q_home, _q_initial, _q_traj;
        double _homing_time;
        
        PositionTask::Ptr _right_arm_position, _right_elbow;
        PositionTask::Ptr _left_arm_position, _left_elbow;
        
    };
    
}
#endif