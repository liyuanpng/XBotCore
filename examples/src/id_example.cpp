#include <id_example.h>

REGISTER_XBOT_PLUGIN_(XBot::IdExample)


XBot::IdExample::IdExample()
{

}

bool XBot::IdExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    _robot->model().getJointPosition(_q0);

    _model = ModelInterface::getModel(handle->getPathToConfigFile());


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
    damp_motion = false;
    torque_ctrl = true;

    _robot->model().getJointPosition(_q0);
    _dq.setConstant(_robot->model().getJointNum(), 0.2);

    _first_loop_time = time;
    _robot->setStiffness(Eigen::VectorXd::Zero(_robot->getJointNum()));
    _robot->setDamping(Eigen::VectorXd::Zero(_robot->getJointNum()));
    _model->syncFrom(*_robot);
}

void XBot::IdExample::control_loop(double time, double period)
{
    static double PERIOD = 1.0;
    static double OMEGA = 2.0 * 3.1415 / PERIOD;

    /* If "damp" command is received, start decreasing OMEGA so as to come to a stop */
    //if(command.read(current_command)){

        if(current_command.str() == "damp"){
            damp_motion = true;
            std::cout << "Damping motion!" << std::endl;
        }

        if(current_command.str() == "impedance_ctrl"){
            _robot->setStiffness(_k0);
            _robot->setDamping(_d0);
            torque_ctrl = false;
            std::cout << "impedance_ctrl!" << std::endl;
        }

        if(current_command.str() == "torque_ctrl"){
            _robot->setStiffness(_k0*0);
            _robot->setDamping(_d0*0);
            torque_ctrl = true;
            std::cout << "torque_ctrl!" << std::endl;
        }
    //}

    if( damp_motion ) OMEGA *= 0.999;

    /* Compute reference trajectory up to 2nd order derivatives */
    _qref = _q0 + _dq * std::sin(OMEGA*(time-_first_loop_time));
    _qdotref = _dq * std::cos(OMEGA*(time-_first_loop_time)) * OMEGA;
    _qddotref = -1.0 * _dq * std::sin(OMEGA*(time-_first_loop_time)) * OMEGA * OMEGA;

    /* Set actual robot positions / velocities to the model used for ID */
    _model->syncFrom_no_update(*_robot, Sync::Position, Sync::Velocity);

    /* Set acceleration reference */
    _model->setJointAcceleration(_qddotref);
    _model->update();

    /* Set tau = B*qddotref + h(q, qdot) */
    _model->computeInverseDynamics(_tau);

    /* Add the PD term */
    _model->getInertiaMatrix(_B);
    _robot->model().getJointPosition(_q);
    _robot->model().getJointVelocity(_qdot);

    double kp = 50, kd = 15;

    _tau += _B*( kp*(_qref-_q) + kd*(_qdotref-_qdot) );

    if(!torque_ctrl) _tau.setZero(_tau.size());

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