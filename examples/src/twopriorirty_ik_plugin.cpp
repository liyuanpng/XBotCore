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
    
    bool TwoPriorityIkPlugin::init_control_plugin(std::string path_to_config_file, 
                                                  XBot::SharedMemory::Ptr shared_memory, 
                                                  XBot::RobotInterface::Ptr robot)
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
        _generator = std::make_shared<JointTrajectoryGenerator>(robot, 10, _q_home);
        
        // Declare a IK solver
        _ik = std::make_shared<TwoPriorityRtIk>(_model);
        
        // Declare a high priority task for the left and for the right arm position
        _right_arm_position = std::make_shared<SixDofTask>(_model, 
                                                             _robot->chain("right_arm").getTipLinkName()
                                                          );
            
        _left_arm_position = std::make_shared<SixDofTask>(_model, 
                                                          _robot->chain("left_arm").getTipLinkName()
                                                         );

        
        _left_elbow = std::make_shared<PositionTask>(_model, 
                                                     _robot->chain("left_arm").getUrdfLinks()[4]->name, 
                                                     Eigen::Vector3d(0,0,0)
                                                     );
        
        _right_elbow = std::make_shared<PositionTask>(_model, 
                                                      _robot->chain("right_arm").getUrdfLinks().at(4)->name, 
                                                      Eigen::Vector3d(0,0,0)
                                                      );
        _left_arm_position->setIkGain(100);
        _right_arm_position->setIkGain(100);
        _left_elbow->setIkGain(1);
        _right_elbow->setIkGain(1);
        
        
         // Add tasks to IK
        _ik->addTaskHighPriority(_right_arm_position);
        _ik->addTaskHighPriority(_left_arm_position);
        
        _ik->addTaskLowPriority(_right_elbow);
        _ik->addTaskLowPriority(_left_elbow);
        
        
        // Get references from shared shared_memory
        _desired_pose_left = shared_memory->get<Eigen::Affine3d>("w_T_left_ee");
        _desired_pose_right = shared_memory->get<Eigen::Affine3d>("w_T_right_ee");
        _desired_pose_left_elb = shared_memory->get<Eigen::Affine3d>("w_T_left_elb");
        _desired_pose_right_elb = shared_memory->get<Eigen::Affine3d>("w_T_right_elb");
        _q_ref = shared_memory->get<Eigen::VectorXd>("q_gen");
        
        

        
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


        _left_arm_position->setReference(*_desired_pose_left);
        _right_arm_position->setReference(*_desired_pose_right);
        _left_elbow->setReference(_desired_pose_left_elb->translation());
        _right_elbow->setReference(_desired_pose_right_elb->translation());
        
        std::cout << "ERROR LEFT: " << _left_arm_position->getError().transpose() << std::endl;
        std::cout << "ERROR RIGHT: " << _right_arm_position->getError().transpose() << std::endl;
	std::cout << "ERROR ELBOW_LEFT: " << _left_elbow->getError().transpose() << std::endl;
	std::cout << "ERROR ELBOW_RIGHT: " << _right_elbow->getError().transpose() << std::endl;
        
        _ik->update(period);
        
        _robot->setReferenceFrom(*_model, XBot::Sync::Position);
//         _robot->setPositionReference(*_q_ref);
        _robot->move();
//         _robot->printTracking();
        
    }
    
    
    bool TwoPriorityIkPlugin::close()
    {
        return true;
    }
    
        
}