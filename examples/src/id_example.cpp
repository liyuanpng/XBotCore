#include <id_example.h>

REGISTER_XBOT_PLUGIN(IdExample, XBot::IdExample)


XBot::IdExample::IdExample()
{

}

bool XBot::IdExample::init_control_plugin(std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory, XBot::RobotInterface::Ptr robot)
{
    _robot = robot;
    _robot->model().getJointPosition(_q0);
    
    _model = ModelInterface::getModel(path_to_config_file);
    
    
    Eigen::VectorXd qmin, qmax;
    _robot->model().getJointLimits(qmin, qmax);
    
    double dqmin, dqmax, dq;
    dqmin = (_q0-qmin).minCoeff();
    dqmax = (qmax-_q0).minCoeff();
    dq = dqmin < dqmax ? dqmin : dqmax;
    
    _dq.setConstant(qmin.size(), dq);
}

void XBot::IdExample::on_start(double time)
{
    _robot->model().getJointPosition(_q0);
    _dq.setConstant(_robot->model().getJointNum(), 0.2);
    
    _first_loop_time = time;
    _robot->setStiffness(Eigen::VectorXd::Zero(_robot->getJointNum()));
    _robot->setDamping(Eigen::VectorXd::Zero(_robot->getJointNum()));
    _model->syncFrom(*_robot);
}

void XBot::IdExample::control_loop(double time, double period)
{
    double PERIOD = 4.0;
    _qref = _q0 + _dq * std::sin(2.0*3.1415*(time-_first_loop_time)/PERIOD);
    _qdotref = _dq * std::cos(2.0*3.1415*(time-_first_loop_time)/PERIOD) / PERIOD;
    _qddotref = -1.0 * _dq * std::sin(2.0*3.1415*(time-_first_loop_time)/PERIOD) / PERIOD / PERIOD;
    
    _model->syncFrom(*_robot, Sync::Position, Sync::Velocity);
    _model->setJointAcceleration(_qddotref); // FIXME syncFrom does the update, but now we need another one!!!!
    _model->update();
    
    _model->computeInverseDynamics(_tau);
    _model->getInertiaMatrix(_B);
    
    _robot->model().getJointPosition(_q);
    _robot->model().getJointVelocity(_qdot);
    
    double kp = 5, kd = 1;
    
    _tau += _B*( kp*(_qref-_q) + kd*(_qdotref-_qdot) );
    
    _model->setJointEffort(_tau);
    _robot->setPositionReference(_qref);
    _robot->setReferenceFrom(*_model, Sync::Effort);
    
    _robot->move();
    
}

void XBot::IdExample::on_stop(double time)
{
}

bool XBot::IdExample::close()
{

}