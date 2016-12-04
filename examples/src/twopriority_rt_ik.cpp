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

#include <twoprioritiy_rt_ik.h>

namespace demo {
 
    Task::Task(std::string name, int size, XBot::ModelInterface::Ptr model):
    _name(name), _model(model), _size(size)
    {
        _A.setZero(size, _model->getJointNum());
        _b.setZero(_model->getJointNum());
        
    }
    
    bool Task::setA(const Eigen::MatrixXd& A)
    {
        if( A.rows() != getSize() || A.cols() != _model->getJointNum() ){
            std::cerr << "ERROR in " << __func__ << "!" << std::endl;
            return false;
        }
        
        _A = A;
        
        return true;
    }
    
    bool Task::setB(const Eigen::VectorXd& b)
    {
        if( b.size() != getSize() ){
            std::cerr << "ERROR in " << __func__ << "!" << std::endl;
            return false;
        }
        
        _b = b;
        
        return true;
    }
    
    
    
    
    
    SixDofTask::SixDofTask(XBot::ModelInterface::Ptr model, std::string link_name):
    Task(std::string("SIXDOF_")+link_name, 6, model), _link_name(link_name), _ik_gain(100), _v_max(100), _omega_max(100)
    {
        _desired.setIdentity();
        _error.setZero(6);
        
    }
    
    
    PositionTask::PositionTask(XBot::ModelInterface::Ptr model, std::string link_name, const Eigen::Vector3d& position):
    Task(std::string("POSITION_")+link_name, 3, model), _link_name(link_name), _ik_gain(100), _v_max(10)
    {
        _desired.setZero();
        _ref.setZero();
        _error.setZero();
        _position.setZero();
    }

    void PositionTask::setReference(const Eigen::Vector3d& ref_position){
        
        _desired = ref_position;

    }

    void PositionTask::update()
    {
        _model->getPointPosition(_link_name, _ref, _position);
        _model->getJacobian(_link_name, _ref, _Jfull);
        _J = _Jfull.topRows(3);
        _error = _desired - _position;

        double velocity_norm = _error.norm()*_ik_gain;
        if( velocity_norm > _v_max ){
            _error *= _v_max/velocity_norm;
        }

        setA(_J);
        setB(_ik_gain*(_desired-_position));
    }

    bool SixDofTask::setReference(const Eigen::Affine3d& ref_pose)
    {
        _desired = ref_pose;
    }
    
    void SixDofTask::update()
    {
        _model->getPose(_link_name, _pose);
        _model->getJacobian(_link_name, _J);
        computeCartesianError(_desired, _pose, _error);

        double velocity_norm = _error.head(3).norm()*_ik_gain;
        if( velocity_norm > _v_max ){
            _error.head(3) *= _v_max/velocity_norm;
        }
        
        double ang_velocity_norm = _error.tail(3).norm()*_ik_gain;
        if( ang_velocity_norm > _omega_max ){
            _error.tail(3) *= _omega_max/ang_velocity_norm;
        }

        setA(_J);
        setB(_ik_gain*_error);
    }
    
    void SixDofTask::computeCartesianError(const Eigen::Affine3d& ref, 
                               const Eigen::Affine3d& actual, 
                               Eigen::VectorXd& error)
    {
        error.resize(6);
        
        Eigen::Quaterniond q(actual.linear()), q_d(ref.linear());
	if(q.dot(q_d) < 0){
	    q.x() *= -1;
	    q.y() *= -1;
	    q.z() *= -1;
	    q.w() *= -1;
	}
        Eigen::Vector3d orientation_error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());
        Eigen::Vector3d position_error = ref.translation() - actual.translation();
        
        error << position_error, orientation_error;
    }
    
    

    
    TwoPriorityRtIk::TwoPriorityRtIk(XBot::ModelInterface::Ptr model):
    _model(model), _hp_size(0), _lp_size(0)
    {
        _q.setZero(model->getJointNum());
        _qdot.setZero(model->getJointNum());
        _q0.setZero(model->getJointNum());
        _eye = Eigen::MatrixXd::Identity(_model->getJointNum(), _model->getJointNum());
    }
    
    bool TwoPriorityRtIk::addTaskHighPriority(Task::Ptr task, double weight)
    {
        
        _hp_tasks.push_back(task);
        _hp_weights.push_back(weight);
        _hp_index.push_back(_hp_size);
        _hp_size += task->getSize();
        
        return true;
        
    }
    
    bool TwoPriorityRtIk::addTaskLowPriority(Task::Ptr task, double weight)
    {
        
        _lp_tasks.push_back(task);
        _lp_weights.push_back(weight);
        _lp_index.push_back(_lp_size);
        _lp_size += task->getSize();
        
        return true;
        
    }
    
    bool TwoPriorityRtIk::update(double dt)
    {
        
        
        if( _hp_size == 0 && _lp_size == 0 ){
            return false;
        }
        
        _model->update();
        _model->getJointPosition(_q);
        
        _J1.setZero(_hp_size, _model->getJointNum());
        _J2.setZero(_lp_size, _model->getJointNum());
        _xdot1.setZero(_hp_size);
        _xdot2.setZero(_lp_size);
        
        // TBD implement weight
        
        for( int i = 0; i < _hp_tasks.size(); i++ ){
            
            Task& task = *_hp_tasks[i];
            task.update();
            _J1.block(_hp_index[i], 0, task.getSize(), _J1.cols()) = task.getA();
            _xdot1.segment(_hp_index[i], task.getSize()) = task.getB();
            
        }
        
        for( int i = 0; i < _lp_tasks.size(); i++ ){
            
            Task& task = *_lp_tasks[i];
            task.update();
            _J2.block(_lp_index[i], 0, task.getSize(), _J2.cols()) = task.getA();
            _xdot2.segment(_lp_index[i], task.getSize()) = task.getB();
            
        }
        
        // NOTE mask torso
        _model->maskJacobian("torso", _J1);
        _model->maskJacobian("torso", _J2);

        _svd1.compute(_J1, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Compute pseudoinverse from SVD 
        _singular_values_1 = _svd1.singularValues();
        std::cout << "SV1: " << _singular_values_1.transpose() << std::endl;
	dampedSingularValueInverse(_singular_values_1);
        
        _J1p.noalias() = _svd1.matrixV()*
			      _singular_values_1.asDiagonal()*
			      _svd1.matrixU().transpose();
			      
        _P1 = (_eye - _J1p*_J1);
            
        if( _lp_size == 0 ){
            
            _qdot = _J1p*_xdot1 + _P1*_q0;
        }
        
        if( _hp_size != 0 && _lp_size != 0 ){
            
            _J2tilde = _J2*_P1;
            _xdot2tilde = _xdot2 - _J2*_J1p*_xdot1; 
            
            _svd2tilde.compute(_J2tilde, Eigen::ComputeThinU | Eigen::ComputeThinV);
            _singular_values_2 = _svd2tilde.singularValues();
            std::cout << "SV2: " << _singular_values_2.transpose() << std::endl;
            dampedSingularValueInverse(_singular_values_2);
            
            _J2p.noalias() = _svd2tilde.matrixV()*
                                _singular_values_2.asDiagonal()*
                                _svd2tilde.matrixU().transpose();
			      
            _P2 = _eye - _J2p*_J2;
            
            _qdot = _J1p*_xdot1 + _P1*(_J2p*_xdot2tilde + _P2*_q0);

        }
        
        if( _hp_size == 0 && _lp_size != 0 ){
            return false;
        }
        
        _model->enforceVelocityLimit(_qdot);
        _q += _qdot * dt;
        _model->enforceJointLimits(_q);
        _model->setJointPosition(_q);
        
            
    }
    
    void TwoPriorityRtIk::dampedSingularValueInverse(Eigen::VectorXd& sv, double threshold)
    {
        for( int i = 0; i < sv.size(); i++ ){
            sv(i) = sv(i) < sv(0)*threshold ? 0 : 1.0/sv(i);
        }
    }
}