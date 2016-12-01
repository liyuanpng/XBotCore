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

#ifndef __XCM_EXAMPLES_TWO_PRIORITY_RT_IK_H__
#define __XCM_EXAMPLES_TWO_PRIORITY_RT_IK_H__

#include <XBotInterface/ModelInterface.h>

// namespace Eigen {
//     typedef Matrix<double, Dynamic, Dynamic, 0, 30, 30> MatrixLd;
// }

namespace demo{
    
    class Task {
        
    public:
        
        typedef std::shared_ptr<Task> Ptr;
        
        Task(std::string name, int size, XBot::ModelInterface::Ptr model);
        
        const Eigen::MatrixXd& getA() const { return _A; }
        const Eigen::VectorXd& getB() const { return _b; }
        const std::string& getName() const { return _name; }
        int getSize() const { return _size; }
        
        virtual void update() {}
        
        
    protected:
        
        bool setA(const Eigen::MatrixXd& A);
        bool setB(const Eigen::VectorXd& b);
        
        XBot::ModelInterface::Ptr _model;
        
        
    private:
        
        Eigen::MatrixXd _A;
        Eigen::VectorXd _b;
        std::string _name;
        int _size;

    };
    
    class SixDofTask : public Task{
      
    public:
        
        typedef std::shared_ptr<SixDofTask> Ptr;
        
        SixDofTask(XBot::ModelInterface::Ptr model, std::string link_name, const Eigen::Vector3d& position);
        
        bool setReference(const Eigen::Affine3d& ref_pose);
        bool setIkGain(double ik_gain){ if( ik_gain > 0 ){ _ik_gain = ik_gain; return true; } }
        void setMaxVelocity(double v_max, double omega_max){ _v_max = v_max; _omega_max = omega_max; }
        const Eigen::Affine3d& getReference() const { return _desired; }
        const Eigen::VectorXd& getError() const { return _error; }
        virtual void update();
        
    private:
        
        void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error);

        Eigen::Affine3d _desired, _pose;
        Eigen::Vector3d _ref, _position;
        Eigen::VectorXd _error;
        Eigen::MatrixXd _J;
        std::string _link_name;
        double _ik_gain;
        double _v_max, _omega_max;
        
        
    };
    
    class PositionTask : public Task{
      
    public:
        
        typedef std::shared_ptr<PositionTask> Ptr;
        
        PositionTask(XBot::ModelInterface::Ptr model, std::string link_name, const Eigen::Vector3d& position);
        
        const Eigen::Vector3d& getControlPoint() const { return _ref; }
        const std::string& getLinkName() const { return _link_name; }
        void setMaxVelocity(double v_max) { _v_max = v_max; }
        bool setIkGain(double ik_gain){ if( ik_gain > 0 ){ _ik_gain = ik_gain; return true; } }
        void setReference(const Eigen::Vector3d& ref_position);
        const Eigen::Vector3d& getReference() const { return _desired; }
        const Eigen::Vector3d& getError() const { return _error; }
        
        virtual void update();
        
    private:
        
        Eigen::Vector3d _ref, _desired, _position;
        Eigen::Vector3d _error;
        Eigen::MatrixXd _Jfull, _J;
        std::string _link_name;
        double _ik_gain;
        double _v_max;
        
        
    };
 
    class TwoPriorityRtIk {
      
    public:

        TwoPriorityRtIk(XBot::ModelInterface::Ptr model);

        public: bool addTaskHighPriority(Task::Ptr task, double weight = 1);
        public: bool addTaskLowPriority(Task::Ptr task, double weight = 1);
        public: bool setPosturalTask(const Eigen::VectorXd& q0);
        public: bool update(double dt);

    protected:

    private:
        
        

        std::vector<Task::Ptr> _hp_tasks, _lp_tasks;
        std::vector<double> _hp_weights, _lp_weights;
        std::vector<int> _hp_index, _lp_index;
        int _hp_size, _lp_size;
        
        Eigen::MatrixXd _J1, _J2, _J1p, _J2p, _P1, _P2;
        Eigen::MatrixXd _J2tilde;
        Eigen::VectorXd _xdot1, _xdot2, _xdot2tilde, _singular_values_1, _singular_values_2;
        Eigen::MatrixXd _eye;
        Eigen::JacobiSVD<Eigen::MatrixXd> _svd1, _svd2tilde;
        Eigen::VectorXd _q, _qdot, _q0;
        XBot::ModelInterface::Ptr _model;

        
    };
    
}

#endif