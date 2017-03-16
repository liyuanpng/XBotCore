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

#include <XCM/XBotXDDP.h>

#include <sys/stat.h>
#include <fcntl.h>

XBot::XBotXDDP::XBotXDDP(std::string config_file)
{
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
    }

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();
    
    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
    }
    
    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    // get the ft
    ft = model.get_ft_sensors();

    // motors
    for(auto& c : robot) {
        for(int i=0; i< c.second.size(); i++) {

            XBot::SubscriberNRT<XBot::RobotState> subscriber_rx(std::string("Motor_id_") + std::to_string(c.second[i]).c_str());
            fd_read[c.second[i]] = subscriber_rx;
            
            XBot::PublisherNRT<XBot::RobotState::pdo_tx> publisher_tx(std::string("rt_in_Motor_id_") + std::to_string(c.second[i]).c_str());
            fd_write[c.second[i]] = publisher_tx;

//             XBot::SubscriberNRT<XBot::sdo_info> subscriber_sdo(std::string("sdo_Motor_id_") + std::to_string(c.second[i]).c_str());
//             fd_sdo_read[c.second[i]] = subscriber_sdo;
          
            // initialize the mutex
            mutex[c.second[i]] = std::make_shared<std::mutex>();
            
            // initialize the pdo_motor
            if(fd_read.count(c.second[i])) {
                XBot::RobotState current_robot_state;
                if( (fd_read[c.second[i]].read(current_robot_state)) ) {
                    pdo_motor[c.second[i]] = std::make_shared<XBot::RobotState>(current_robot_state);
                }
            }

            // initialize the sdo info
//             if(fd_sdo_read.count(c.second[i])) {
//                 XBot::sdo_info current_sdo_info;
//                 if( fd_sdo_read[c.second[i]].read(current_sdo_info) ) {
//                     sdo_info[c.second[i]] = std::make_shared<XBot::sdo_info>(current_sdo_info);
//                 }
//             }

        }
    }
    
    //ft
    for(auto& ft_j : ft) {
        // initialize all the fd reading for the ft
        XBot::SubscriberNRT<XBot::RobotFT::pdo_rx> subscriber_ft(std::string("Ft_id_") + std::to_string(ft_j.second).c_str());
        fd_ft_read[ft_j.second] = subscriber_ft;

        // initialize the mutex
        mutex[ft_j.second] = std::make_shared<std::mutex>();
    }

}

std::map< std::string, std::vector< int > > XBot::XBotXDDP::get_robot_map()
{
    return model.get_robot();
}

std::map< std::string, int > XBot::XBotXDDP::get_ft_sensors_map()
{
    return model.get_ft_sensors();
}


XBot::XBotCoreModel XBot::XBotXDDP::get_robot_model()
{
    return model;
}



bool XBot::XBotXDDP::init()
{
   DPRINTF("XBotXDDP INIT\n");
   return true;
}

void XBot::XBotXDDP::update()
{  
    for( auto& f: fd_read) {
        mutex.at(f.first)->lock();
        
        // write to the NRT publisher to command the RobotStateTX in the pdo_motor buffer
        XBot::RobotState::pdo_tx actual_pdo_tx = pdo_motor.at(f.first)->RobotStateTX;
        fd_write.at(f.first).write(actual_pdo_tx);
        
        // NOTE the single joint element can only be controlled by either the RT or the N-RT!
        
        // reading from the NRT subscriber pipes to update the RobotStateRX in the pdo_motor buffer 
        fd_read.at(f.first).read(_actual_pdo_motor);
        (*pdo_motor.at(f.first)).RobotStateRX = _actual_pdo_motor.RobotStateRX;
        
//         std::cout << "ID: " << f.first << " - link_pos :  " << (*pdo_motor.at(f.first)).RobotStateRX.link_pos << std::endl;
        
        mutex.at(f.first)->unlock();
    }

}

bool XBot::XBotXDDP::get_min_pos(int joint_id, float& min_pos)
{
    if( sdo_info.count(joint_id) ){
        min_pos = sdo_info.at(joint_id)->min_pos;
        return true;
    }
    min_pos = 0.0; // NOTE avoid to break the robot
    return false;
}

bool XBot::XBotXDDP::get_max_pos(int joint_id, float& max_pos)
{
    if( sdo_info.count(joint_id) ){
        max_pos = sdo_info.at(joint_id)->max_pos;
        return true;
    }
    max_pos = 0.0; // NOTE avoid to break the robot
    return false;
}

bool XBot::XBotXDDP::get_ctrl_status_cmd(int joint_id, uint16_t& ctrl_status_cmd)
{
    if( sdo_info.count(joint_id) ){
        ctrl_status_cmd = sdo_info.at(joint_id)->ctrl_status_cmd;
        return true;
    }
    return false;
}



bool XBot::XBotXDDP::get_link_pos(int joint_id, float& link_pos)
{
    mutex.at(joint_id)->lock();
    link_pos = pdo_motor.at(joint_id)->RobotStateRX.link_pos;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_motor_pos(int joint_id, float& motor_pos)
{
    mutex.at(joint_id)->lock();
    motor_pos = pdo_motor.at(joint_id)->RobotStateRX.motor_pos;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_link_vel(int joint_id, int16_t& link_vel)
{
    mutex.at(joint_id)->lock();
    link_vel = pdo_motor.at(joint_id)->RobotStateRX.link_vel;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_motor_vel(int joint_id, int16_t& motor_vel)
{
    mutex.at(joint_id)->lock();
    motor_vel = pdo_motor.at(joint_id)->RobotStateRX.motor_vel;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_torque(int joint_id, float& torque)
{
    mutex.at(joint_id)->lock();
    torque = pdo_motor.at(joint_id)->RobotStateRX.torque;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_temperature(int joint_id, uint16_t& temperature)
{
    mutex.at(joint_id)->lock();
    temperature = pdo_motor.at(joint_id)->RobotStateRX.temperature;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_fault(int joint_id, uint16_t& fault)
{
    mutex.at(joint_id)->lock();
    fault = pdo_motor.at(joint_id)->RobotStateRX.fault;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_rtt(int joint_id, uint16_t& rtt)
{
    mutex.at(joint_id)->lock();
    rtt = pdo_motor.at(joint_id)->RobotStateRX.rtt;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_op_idx_ack(int joint_id, uint16_t& op_idx_ack)
{
    mutex.at(joint_id)->lock();
    op_idx_ack = pdo_motor.at(joint_id)->RobotStateRX.op_idx_ack;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_aux(int joint_id, float& aux)
{
    mutex.at(joint_id)->lock();
    aux = pdo_motor.at(joint_id)->RobotStateRX.aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_gains(int joint_id, std::vector< uint16_t >& gain_vector)
{
    mutex.at(joint_id)->lock();
    // resize the gain vector
    gain_vector.resize(5);  
    gain_vector[0] = pdo_motor.at(joint_id)->RobotStateTX.gain_0;
    gain_vector[1] = pdo_motor.at(joint_id)->RobotStateTX.gain_1;
    gain_vector[2] = pdo_motor.at(joint_id)->RobotStateTX.gain_2;
    gain_vector[3] = pdo_motor.at(joint_id)->RobotStateTX.gain_3;
    gain_vector[4] = pdo_motor.at(joint_id)->RobotStateTX.gain_4;
    mutex.at(joint_id)->unlock();
    return true;
}


bool XBot::XBotXDDP::set_pos_ref(int joint_id, const float& pos_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.pos_ref = pos_ref;
    //DPRINTF("joint : %d - set pos ref : %f\n", joint_id, pdo_motor.at(joint_id)->RobotStateTX.pos_ref);
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_vel_ref(int joint_id, const int16_t& vel_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.vel_ref = vel_ref;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_tor_ref(int joint_id, const int16_t& tor_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.tor_ref = tor_ref;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_gains(int joint_id, const std::vector< uint16_t >& gains)
{
    mutex.at(joint_id)->lock();
    if(gains.size() == 5) {
        pdo_motor.at(joint_id)->RobotStateTX.gain_0 = gains[0];
        pdo_motor.at(joint_id)->RobotStateTX.gain_1 = gains[1];
        pdo_motor.at(joint_id)->RobotStateTX.gain_2 = gains[2];
        pdo_motor.at(joint_id)->RobotStateTX.gain_3 = gains[3];
        pdo_motor.at(joint_id)->RobotStateTX.gain_4 = gains[4];
    }
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_fault_ack(int joint_id, const int16_t& fault_ack)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.fault_ack = fault_ack;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_ts(int joint_id, const uint16_t& ts)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.ts = ts;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.op_idx_aux = op_idx_aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::set_aux(int joint_id, const float& aux)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->RobotStateTX.aux = aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotXDDP::get_ft(int ft_id, std::vector< float >& ft, int channels)
{
    mutex.at(ft_id)->lock();
    
    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;  
    fd_ft_read.at(ft_id).read(actual_pdo_rx_ft);
    
    ft.resize(channels);
    std::memcpy(ft.data(), &(actual_pdo_rx_ft.force_X), channels*sizeof(float));
    
    mutex.at(ft_id)->unlock();
    
    return true;  
}

bool XBot::XBotXDDP::get_ft_fault(int ft_id, uint16_t& fault)
{
    mutex.at(ft_id)->lock();
    
    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;   
    fd_ft_read.at(ft_id).read(actual_pdo_rx_ft);
    
    fault = actual_pdo_rx_ft.fault;
    
    mutex.at(ft_id)->unlock();
    
    return true;  
}

bool XBot::XBotXDDP::get_ft_rtt(int ft_id, uint16_t& rtt)
{
    mutex.at(ft_id)->lock();
    
    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;    
    fd_ft_read.at(ft_id).read(actual_pdo_rx_ft);
    
    rtt = actual_pdo_rx_ft.rtt;
    
    mutex.at(ft_id)->unlock();
    
    return true;  
}



XBot::XBotXDDP::~XBotXDDP()
{
    printf("~XBotXDDP()\n");
}
