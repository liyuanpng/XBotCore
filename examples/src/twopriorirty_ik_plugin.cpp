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

#include <twopriority_ik_plugin.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(TwoPriorityRtIkPlugin_factory, demo::TwoPriorityIkPlugin, XBot::XBotControlPlugin);

namespace demo {
    
    bool TwoPriorityIkPlugin::init_control_plugin(std::string path_to_config_file, XBot::RobotInterface::Ptr robot)
    {
        // Save the robot and obtain a couple of models
        _robot = robot;
        _model = XBot::ModelInterface::getModel(path_to_config_file);
        _model_traj = XBot::ModelInterface::getModel(path_to_config_file);
        
        // Get the homing configuration from SRDF
        _robot->getRobotState("home", _q_home);
        
        // Read robot state and set the inital configuration
        _robot->sense();
        _robot->getJointPosition(_q_initial);
        _homing_time = 5;
        
        // Set the model for IK to home position
        _model->setJointPosition(_q_home);
        _model->update();
        
        // Declare a joint trajectory generator which starts from home
        _generator = std::make_shared<JointTrajectoryGenerator>(robot, 20, _q_home);
        
        // Declare a IK solver
        _ik = std::make_shared<TwoPriorityRtIk>(_model);
        
        // Declare a high priority task for the left and for the right arm position
        _right_arm_position = std::make_shared<SixDofTask>(_model, 
                                                             _robot->chain("right_arm").getTipLinkName(), 
                                                             Eigen::Vector3d(0,0,0)
                                                             );
            
        _left_arm_position = std::make_shared<SixDofTask>(_model, 
                                                            _robot->chain("left_arm").getTipLinkName(), 
                                                            Eigen::Vector3d(0,0,0)
                                                            );
        
	
        
        
        _left_elbow = std::make_shared<PositionTask>(_model, 
                                                     _robot->chain("left_arm").getUrdfLinks()[4]->name, 
                                                     Eigen::Vector3d(0,0,0)
                                                     );
        
        _right_elbow = std::make_shared<PositionTask>(_model, 
                                                      _robot->chain("right_arm").getUrdfLinks().at(4)->name, 
                                                      Eigen::Vector3d(0,0,0)
                                                      );
        _left_arm_position->setIkGain(10);
        _right_arm_position->setIkGain(10);
        _left_elbow->setIkGain(10);
        _right_elbow->setIkGain(10);
        
        
         // Add tasks to IK
        _ik->addTaskHighPriority(_right_arm_position);
        _ik->addTaskHighPriority(_left_arm_position);
        
//         _ik->addTaskLowPriority(_right_elbow);
//         _ik->addTaskLowPriority(_left_elbow);
        
        return true;
        
    }
    
    
    void TwoPriorityIkPlugin::control_loop(double time, double period)
    {
//         period = 0.0005;
        std::cout << "Period: " << period << std::endl;
        _robot->sense();
    
        // Go to homing
        if( (time - get_first_loop_time()) <= _homing_time ){
            _robot->setPositionReference(_q_initial + 0.5*(1-std::cos(3.1415*(time - get_first_loop_time())/_homing_time))*(_q_home-_q_initial));
            _robot->move();
            return;
        }
        
        // Get current config
        _generator->getQ(period, _q_traj);
        _model_traj->setJointPosition(_q_traj);
        _model_traj->update();
        
	Eigen::Vector3d desired_position_elbow_l, desired_position_elbow_r;
	Eigen::Affine3d desired_pose_left, desired_pose_right;
	
        _model_traj->getPose(_left_arm_position->getLinkName(), 
                                       				// TBD write get pose with ref point , 
                                      desired_pose_left);
        
        _model_traj->getPose(_right_arm_position->getLinkName(), 
                                     
                                      desired_pose_right);
        
        _model_traj->getPointPosition(_right_elbow->getLinkName(), 
                                      _right_elbow->getControlPoint(), 
                                      desired_position_elbow_r);
        
        _model_traj->getPointPosition(_left_elbow->getLinkName(), 
                                      _left_elbow->getControlPoint(), 
                                      desired_position_elbow_l);
//         
        _left_arm_position->setReference(desired_pose_left);
        _right_arm_position->setReference(desired_pose_right);
        _left_elbow->setReference(desired_position_elbow_l);
        _right_elbow->setReference(desired_position_elbow_r);
        
//         std::cout << "ERROR LEFT: " << _left_arm_position->getError().transpose() << std::endl;
//         std::cout << "ERROR RIGHT: " << _right_arm_position->getError().transpose() << std::endl;
// 	std::cout << "ERROR ELBOW_LEFT: " << _left_elbow->getError().transpose() << std::endl;
// 	std::cout << "ERROR ELBOW_RIGHT: " << _right_elbow->getError().transpose() << std::endl;
        
        _ik->update(period);
        
        _robot->setReferenceFrom(*_model, XBot::Sync::Position);
        _robot->move();
//         _robot->printTracking();
        
    }
    
    
    bool TwoPriorityIkPlugin::close()
    {
        return true;
    }
    
        
}