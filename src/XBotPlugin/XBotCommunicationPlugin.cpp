/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <XBotPlugin/XBotCommunicationPlugin.h>

REGISTER_XBOT_PLUGIN_(XBot::XBotCommunicationPlugin)

XBot::XBotCommunicationPlugin::XBotCommunicationPlugin()
{

}

bool XBot::XBotCommunicationPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    // get the robot
    _robot = handle->getRobotInterface();

    // initialize filter
    int cutoff_freq = 1.0;
    _filter_q = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*cutoff_freq, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    _filter_k = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*cutoff_freq, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    _filter_d = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*cutoff_freq, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));
    _filter_qdot = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>(2*3.1415*cutoff_freq, 1.0, 0.001, Eigen::VectorXd::Zero(_robot->getJointNum()));

    
    // NOTE filter ON by default
    _filter_enabled = true;
    _robot->getMotorPosition(_qref);
    _robot->getStiffness(_kref);
    _robot->getDamping(_dref);
    _robot->getJointVelocity(_qdotref);
 
    Logger::warning() << "Filter ON by default, cutoff frequency is " << cutoff_freq << " Hz" << Logger::endl();

     // intiliaze shared memory
    _pos_ref_map = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("pos_ref_map_so");
    _vel_ref_map = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("vel_ref_map_so");
    _tor_ref_map = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("tor_ref_map_so");
    _k_ref_map = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("k_ref_map_so");
    _d_ref_map = handle->getSharedMemory()->getSharedObject<XBot::JointIdMap>("d_ref_map_so");
    
    return true;
}

void XBot::XBotCommunicationPlugin::on_start(double time)
{
    _start_time = time;
    _robot->getMotorPosition(_q0);
    _robot->getStiffness(_k0);
    _robot->getDamping(_d0);
    _robot->getJointVelocity(_qdot0);

    _filter_q.reset(_q0);
    _filter_k.reset(_k0);
    _filter_d.reset(_d0);
    _filter_qdot.reset(_qdot0 * 0.0);

}

void XBot::XBotCommunicationPlugin::on_stop(double time)
{

}


void XBot::XBotCommunicationPlugin::control_loop(double time, double period)
{

    //if(command.read(current_command)){
        if(current_command.str() == "filter ON"){
            _filter_q.setOmega(2*3.1415*1.0);
            _filter_k.setOmega(2*3.1415*1.0);
            _filter_d.setOmega(2*3.1415*1.0);
            _filter_qdot.setOmega(2*3.1415*1.0);
        }
        if(current_command.str() == "filter OFF"){
            _filter_q.setOmega(2*3.1415*200);
            _filter_k.setOmega(2*3.1415*200);
            _filter_d.setOmega(2*3.1415*200);
            _filter_qdot.setOmega(2*3.1415*200);
        }
    //}
    
    // read from shared memory the ref maps and set them

    _robot->setPositionReference(_pos_ref_map.get());
    _robot->setVelocityReference(_vel_ref_map.get());
    _robot->setEffortReference(_tor_ref_map.get());
    _robot->setStiffness(_k_ref_map.get());
    _robot->setDamping(_d_ref_map.get());

    double alpha = (time - _start_time) / 5.0;

    if( alpha < 1 ){
        _robot->getPositionReference(_qref);
        _qref = alpha*_qref + (1-alpha)*_q0;
        _robot->setPositionReference(_qref);
        
        _robot->getVelocityReference(_qdotref);
        _qdotref = alpha*_qdotref + (1-alpha)*_qdot0;
        _robot->setVelocityReference(_qdotref);

        _robot->getStiffness(_kref);
        _kref = alpha*_kref + (1-alpha)*_k0;
        _robot->setStiffness(_kref);

        _robot->getDamping(_dref);
        _dref = alpha*_dref + (1-alpha)*_d0;
        _robot->setDamping(_dref);
    }

    if(_filter_enabled){
        _robot->getPositionReference(_qref);
        _robot->setPositionReference(_filter_q.process(_qref));

        _robot->getStiffness(_kref);
        _robot->setStiffness(_filter_k.process(_kref));

        _robot->getDamping(_dref);
        _robot->setDamping(_filter_d.process(_dref));
        
        _robot->getVelocityReference(_qdotref);
        _robot->setVelocityReference(_filter_qdot.process(_qdotref));
    }

    _robot->move();

}

bool XBot::XBotCommunicationPlugin::close(void)
{

    return true;
}

XBot::XBotCommunicationPlugin::~XBotCommunicationPlugin()
{
    Logger::info() << "~XBotCommunicationPlugin()" << Logger::endl();
}


