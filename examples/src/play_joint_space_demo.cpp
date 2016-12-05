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

#include <play_joint_space_demo.h>

SHLIBPP_DEFINE_SHARED_SUBCLASS(PlayJointSpaceDemo_factory, demo::PlayJointSpaceDemo, XBot::XBotControlPlugin);

namespace demo {
    
bool PlayJointSpaceDemo::init_control_plugin(std::string path_to_config_file, 
                                             XBot::SharedMemory::Ptr shared_memory, 
                                             XBot::RobotInterface::Ptr robot)
{
    _robot = robot;
    _model = XBot::ModelInterface::getModel(path_to_config_file);
    _shared_memory = shared_memory;
    
    // Advertise shared objects
    _q_gen = shared_memory->advertise<Eigen::VectorXd>("q_gen");
    _T_left_ee = shared_memory->advertise<Eigen::Affine3d>("w_T_left_ee");
    _T_right_ee = shared_memory->advertise<Eigen::Affine3d>("w_T_right_ee");
    _T_left_elb = shared_memory->advertise<Eigen::Affine3d>("w_T_left_elb");
    _T_right_elb = shared_memory->advertise<Eigen::Affine3d>("w_T_right_elb");
    
    // Allocate shared objects
    _q_gen.reset(new Eigen::VectorXd);
    _T_left_ee.reset(new Eigen::Affine3d);
    _T_right_ee.reset(new Eigen::Affine3d);
    _T_left_elb.reset(new Eigen::Affine3d);
    _T_right_elb.reset(new Eigen::Affine3d);
    
    robot->getRobotState("home", _q_home);
    _generator = std::make_shared<JointTrajectoryGenerator>(robot, 10, _q_home);
    
    _robot->sense();
    _robot->getJointPosition(_q0);
    _homing_time = 5;
    return true;
}

void PlayJointSpaceDemo::control_loop(double time, double period)
{
    _robot->sense();
    
    if( (time - get_first_loop_time()) <= _homing_time ){
        _robot->setPositionReference(_q0 + 0.5*(1-std::cos(3.1415*(time - get_first_loop_time())/_homing_time))*(_q_home-_q0));
        _robot->move();
        return;
    }
    
    _generator->getQ(period, _q);
    *_q_gen = _q;
    _model->setJointPosition(_q);
    _model->update();
    
    _model->getPose(_robot->chain("right_arm").getTipLinkName(), *_T_right_ee);
    _model->getPose(_robot->chain("left_arm").getTipLinkName(), *_T_left_ee);
    _model->getPose(_robot->chain("left_arm").getUrdfLinks()[4]->name, *_T_left_elb);
    _model->getPose(_robot->chain("right_arm").getUrdfLinks()[4]->name, *_T_right_elb);
 

    
    
//     _robot->setPositionReference(_q);
    
//     _robot->printTracking();
//     _robot->move();
}

bool PlayJointSpaceDemo::close()
{
    return true;
}

    
}