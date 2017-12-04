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

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["XBotInterface"];

    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    computeAbsolutePath(urdf_path, "/", urdf_path);

    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    computeAbsolutePath(srdf_path, "/", srdf_path);

    joint_map_config = x_bot_core_config["joint_map_path"].as<std::string>();
    computeAbsolutePath(joint_map_config, "/", joint_map_config);

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
    // get the imu
    imu = model.get_imu_sensors();
    // get the hand
    hand = model.get_hands();

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
//             mutex[c.second[i]] = std::make_shared<std::mutex>();

            // initialize the pdo_motor
            if(fd_read.count(c.second[i])) {
                XBot::RobotState current_robot_state;
                while( !(fd_read[c.second[i]].read(current_robot_state)) ) {
                   sleep(1);
                }
                pdo_motor[c.second[i]] = std::make_shared<XBot::RobotState>(current_robot_state);
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
//         mutex[ft_j.second] = std::make_shared<std::mutex>();
    }

    //imu
    for(auto& imu_j : imu) {
        // initialize all the fd reading for the imu
        XBot::SubscriberNRT<XBot::RobotIMU::pdo_rx> subscriber_imu(std::string("Imu_id_") + std::to_string(imu_j.second).c_str());
        fd_imu_read[imu_j.second] = subscriber_imu;

    }

    //hand
    for(auto& hand_j : hand) {
            XBot::SubscriberNRT<XBot::RobotState> subscriber_rx(std::string("Motor_id_") + std::to_string(hand_j.second).c_str());
            fd_read[hand_j.second] = subscriber_rx;

            XBot::PublisherNRT<XBot::RobotState::pdo_tx> publisher_tx(std::string("rt_in_Motor_id_") + std::to_string(hand_j.second).c_str());
            fd_write[hand_j.second] = publisher_tx;

//             XBot::SubscriberNRT<XBot::sdo_info> subscriber_sdo(std::string("sdo_Motor_id_") + std::to_string(c.second[i]).c_str());
//             fd_sdo_read[c.second[i]] = subscriber_sdo;

            // initialize the mutex
//             mutex[c.second[i]] = std::make_shared<std::mutex>();

            // initialize the pdo_motor
            if(fd_read.count(hand_j.second)) {
                XBot::RobotState current_robot_state;
                while( !(fd_read[hand_j.second].read(current_robot_state)) ) {
                   sleep(1);
                }
                pdo_motor[hand_j.second] = std::make_shared<XBot::RobotState>(current_robot_state);
            }
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
   return true;
}

void XBot::XBotXDDP::updateTX()
{
    // Motor + hands
    for( auto& f: fd_read) {

        // write to the NRT publisher to command the RobotStateTX in the pdo_motor buffer
        XBot::RobotState::pdo_tx actual_pdo_tx = pdo_motor.at(f.first)->RobotStateTX;
        fd_write.at(f.first).write(actual_pdo_tx);

    }
}

void XBot::XBotXDDP::updateRX()
{
    // Motor + hands
    for( auto& f: fd_read) {
        // NOTE the single joint element can only be controlled by either the RT or the N-RT!

        // reading from the NRT subscriber pipes to update the RobotStateRX in the pdo_motor buffer
        fd_read.at(f.first).read(*pdo_motor.at(f.first));

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



bool XBot::XBotXDDP::get_link_pos(int joint_id, double& link_pos)
{
    link_pos = pdo_motor.at(joint_id)->RobotStateRX.link_pos;
    return true;
}

bool XBot::XBotXDDP::get_motor_pos(int joint_id, double& motor_pos)
{
    motor_pos = pdo_motor.at(joint_id)->RobotStateRX.motor_pos;
    return true;
}

bool XBot::XBotXDDP::get_link_vel(int joint_id, double& link_vel)
{
    link_vel = pdo_motor.at(joint_id)->RobotStateRX.link_vel;
    return true;
}

bool XBot::XBotXDDP::get_motor_vel(int joint_id, double& motor_vel)
{
    motor_vel = pdo_motor.at(joint_id)->RobotStateRX.motor_vel;
    return true;
}

bool XBot::XBotXDDP::get_torque(int joint_id, double& torque)
{
    torque = pdo_motor.at(joint_id)->RobotStateRX.torque;
    return true;
}

bool XBot::XBotXDDP::get_temperature(int joint_id, double& temperature)
{
    temperature = pdo_motor.at(joint_id)->RobotStateRX.temperature;
    return true;
}

bool XBot::XBotXDDP::get_fault(int joint_id, double& fault)
{
    fault = pdo_motor.at(joint_id)->RobotStateRX.fault;
    return true;
}

bool XBot::XBotXDDP::get_rtt(int joint_id, double& rtt)
{
    rtt = pdo_motor.at(joint_id)->RobotStateRX.rtt;
    return true;
}

bool XBot::XBotXDDP::get_op_idx_ack(int joint_id, double& op_idx_ack)
{
    op_idx_ack = pdo_motor.at(joint_id)->RobotStateRX.op_idx_ack;
    return true;
}

bool XBot::XBotXDDP::get_aux(int joint_id, double& aux)
{
    aux = pdo_motor.at(joint_id)->RobotStateRX.aux;
    return true;
}

bool XBot::XBotXDDP::get_gains(int joint_id, std::vector< double >& gain_vector)
{
    // resize the gain vector
    gain_vector.resize(5);
    gain_vector[0] = pdo_motor.at(joint_id)->RobotStateTX.gain_0;
    gain_vector[1] = pdo_motor.at(joint_id)->RobotStateTX.gain_1;
    gain_vector[2] = pdo_motor.at(joint_id)->RobotStateTX.gain_2;
    gain_vector[3] = pdo_motor.at(joint_id)->RobotStateTX.gain_3;
    gain_vector[4] = pdo_motor.at(joint_id)->RobotStateTX.gain_4;
    return true;
}

bool XBot::XBotXDDP::get_pos_ref(int joint_id, double& pos_ref)
{
    pos_ref = pdo_motor.at(joint_id)->RobotStateTX.pos_ref;
    return true;
}

bool XBot::XBotXDDP::get_vel_ref(int joint_id, double& vel_ref)
{
    vel_ref = pdo_motor.at(joint_id)->RobotStateTX.vel_ref;
    return true;
}

bool XBot::XBotXDDP::get_tor_ref(int joint_id, double& tor_ref)
{
    tor_ref = pdo_motor.at(joint_id)->RobotStateTX.tor_ref;
    return true;
}


bool XBot::XBotXDDP::set_pos_ref(int joint_id, const double& pos_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.pos_ref = pos_ref;
    //DPRINTF("joint : %d - set pos ref : %f\n", joint_id, pdo_motor.at(joint_id)->RobotStateTX.pos_ref);
    return true;
}

bool XBot::XBotXDDP::set_vel_ref(int joint_id, const double& vel_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.vel_ref = vel_ref;
    return true;
}

bool XBot::XBotXDDP::set_tor_ref(int joint_id, const double& tor_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.tor_ref = tor_ref;
    return true;
}

bool XBot::XBotXDDP::set_gains(int joint_id, const std::vector< double >& gains)
{
    if(gains.size() == 5) {
        pdo_motor.at(joint_id)->RobotStateTX.gain_0 = gains[0];
        pdo_motor.at(joint_id)->RobotStateTX.gain_1 = gains[1];
        pdo_motor.at(joint_id)->RobotStateTX.gain_2 = gains[2];
        pdo_motor.at(joint_id)->RobotStateTX.gain_3 = gains[3];
        pdo_motor.at(joint_id)->RobotStateTX.gain_4 = gains[4];
    }
    return true;
}

bool XBot::XBotXDDP::set_fault_ack(int joint_id, const double& fault_ack)
{
    pdo_motor.at(joint_id)->RobotStateTX.fault_ack = fault_ack;
    return true;
}

bool XBot::XBotXDDP::set_ts(int joint_id, const double& ts)
{
    pdo_motor.at(joint_id)->RobotStateTX.ts = ts;
    return true;
}

bool XBot::XBotXDDP::set_op_idx_aux(int joint_id, const double& op_idx_aux)
{
    pdo_motor.at(joint_id)->RobotStateTX.op_idx_aux = op_idx_aux;
    return true;
}

bool XBot::XBotXDDP::set_aux(int joint_id, const double& aux)
{
    pdo_motor.at(joint_id)->RobotStateTX.aux = aux;
    return true;
}

bool XBot::XBotXDDP::get_ft(int ft_id, std::vector< double >& ft, int channels)
{

    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;
    if(!fd_ft_read.at(ft_id).read(actual_pdo_rx_ft)){
        return false;
    }

    ft.resize(channels);
    std::memcpy(ft.data(), &(actual_pdo_rx_ft.force_X), channels*sizeof(double));

    return true;
}

bool XBot::XBotXDDP::get_ft_fault(int ft_id, double& fault)
{
    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;
    if(!fd_ft_read.at(ft_id).read(actual_pdo_rx_ft)){
        return false;
    }

    fault = actual_pdo_rx_ft.fault;

    return true;
}

bool XBot::XBotXDDP::get_ft_rtt(int ft_id, double& rtt)
{
    XBot::RobotFT::pdo_rx actual_pdo_rx_ft;
    if(!fd_ft_read.at(ft_id).read(actual_pdo_rx_ft)){
        return false;
    }

    rtt = actual_pdo_rx_ft.rtt;

    return true;
}

bool XBot::XBotXDDP::get_imu(int imu_id,
                             std::vector< double >& lin_acc,
                             std::vector< double >& ang_vel,
                             std::vector< double >& quaternion)
{

    XBot::RobotIMU::pdo_rx actual_pdo_rx_imu;
    if(!fd_imu_read.at(imu_id).read(actual_pdo_rx_imu)){
        return false;
    }

    if(quaternion.size() == 4) {
        quaternion.at(0) = actual_pdo_rx_imu.quat_X;
        quaternion.at(1) = actual_pdo_rx_imu.quat_Y;
        quaternion.at(2) = actual_pdo_rx_imu.quat_Z;
        quaternion.at(3) = actual_pdo_rx_imu.quat_W;
    }

    if(ang_vel.size() == 3 ) {
        ang_vel.at(0) = actual_pdo_rx_imu.ang_vel_X;
        ang_vel.at(1) = actual_pdo_rx_imu.ang_vel_Y;
        ang_vel.at(2) = actual_pdo_rx_imu.ang_vel_Z;
    }

    if(lin_acc.size() == 3 ) {
        lin_acc.at(0) = actual_pdo_rx_imu.lin_acc_X;
        lin_acc.at(1) = actual_pdo_rx_imu.lin_acc_Y;
        lin_acc.at(2) = actual_pdo_rx_imu.lin_acc_Z;
    }

    return true;

}

bool XBot::XBotXDDP::get_imu_fault(int imu_id, double& fault)
{
    XBot::RobotIMU::pdo_rx actual_pdo_rx_imu;
    if(!fd_imu_read.at(imu_id).read(actual_pdo_rx_imu)){
        return false;
    }

    fault = actual_pdo_rx_imu.fault;

    return true;
}

bool XBot::XBotXDDP::get_imu_rtt(int imu_id, double& rtt)
{
    XBot::RobotIMU::pdo_rx actual_pdo_rx_imu;
    if(!fd_imu_read.at(imu_id).read(actual_pdo_rx_imu)){
        return false;
    }

    rtt = actual_pdo_rx_imu.rtt;

    return true;
}

bool XBot::XBotXDDP::grasp(int hand_id, double grasp_percentage)
{
    // HACK 12.0 is assumed as maximum position range
    pdo_motor.at(hand_id)->RobotStateTX.pos_ref = -1 + grasp_percentage * 12.0;
    return true;
}

double XBot::XBotXDDP::get_grasp_state(int hand_id)
{
    double grasp_state = 0.0;
    double link_pos = pdo_motor.at(hand_id)->RobotStateRX.link_pos;

    if( link_pos != 0.0) {
        // HACK 12.0 is assumed as maximum position range
        grasp_state = (1 + link_pos) / 12.0;
    }
    return  grasp_state;
}


XBot::XBotXDDP::~XBotXDDP()
{
//     Logger::info() << "~XBotXDDP()" << Logger::endl();
}

bool XBot::XBotXDDP::computeAbsolutePath (  const std::string& input_path,
                                            const std::string& middle_path,
                                            std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}
