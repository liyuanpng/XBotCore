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

#include <ik_example.h>

REGISTER_XBOT_PLUGIN_(XBot::IkExample)

void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error);

void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error)
{
    error.resize(6);

    Eigen::Quaterniond q(actual.linear()), q_d(ref.linear());
    Eigen::Vector3d orientation_error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());
    Eigen::Vector3d position_error = ref.translation() - actual.translation();

    error << position_error, orientation_error;
}


namespace XBot {

IkExample::IkExample()
{

}

bool IkExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
    _model = ModelInterface::getModel(handle->getPathToConfigFile());

    _robot->getRobotState("home", _q_home);
    _robot->sense();
    _robot->getJointPosition(_q0);

    _alpha = 0;
    _homing_time = 5;
    _ik_started = false;

    _end_effector = _robot->chain("left_arm").getTipLinkName();
    _length = 0.3;
    _period = 6;
    return true;
}

void IkExample::on_start(double time)
{
    _first_loop_time = time;
}

void IkExample::on_stop(double time)
{

}


void IkExample::control_loop(double time, double period)
{

    if( (time - _first_loop_time) <= _homing_time ){
        _robot->setPositionReference(_q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0));
        _robot->move();
        _robot->sense();
        return;
    }

    if( !_ik_started ){
//         _robot->sense();
//         _model->syncFrom(*_robot);
	_model->setJointPosition(_q_home);
	_model->update();
        _model->getPose(_end_effector, _initial_pose);
        _model->getJointPosition(_q);
        _desired_pose = _initial_pose;
	_ik_started_time = time;
        _ik_started = true;
    }

    double dt = period;

    // Set the desired end-effector pose at current time
    _desired_pose.linear() = _initial_pose.linear();
    _desired_pose.translation() = _initial_pose.translation() + Eigen::Vector3d(0,0,1)*0.5*_length*(1-std::cos(2*3.1415/_period*(time-_ik_started_time)));
    // Compute the pose corresponding to the model state
    _model->getPose(_end_effector, _actual_pose);

    // Compute the cartesian error
    computeCartesianError(_desired_pose, _actual_pose, _cartesian_error);

    // Set a cartesian velocity which is proportional to the error
    double ik_gain = 100;
    _xdot = ik_gain * _cartesian_error;

    // Compute the jacobian matrix
    _model->getJacobian(_end_effector, _J);
    _model->maskJacobian("torso", _J); // We don't want the torso to move

    // Compute the required joint velocities
    _qdot = _J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(_xdot);

    // Integrate the computed velocities
    _q += _qdot * dt;

    // Update the model
    _model->setJointPosition(_q);
    _model->update();

    // Use the model state as a reference for controllers
    _robot->setReferenceFrom(*_model, Sync::Position);
    _robot->move();


//     _robot->sense();
//     _robot->printTracking();

}

bool IkExample::close()
{
    return true;
}






}