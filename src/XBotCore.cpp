/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#include <XBotCore/XBotCore.h>

#include <boost/bind.hpp>

XBot::XBotCore::XBotCore(const char* config_yaml) : 
    XBotEcat(config_yaml), 
    _path_to_config(config_yaml)
{

}



void XBot::XBotCore::control_init(void) 
{
    
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(this);
    std::shared_ptr<XBot::IXBotFT> xbot_ft(this);
    std::shared_ptr<XBot::IXBotIMU> xbot_imu(this);
    std::shared_ptr<XBot::IXBotHand> xbot_hand(this);
    
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    
    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::XBotCore::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider, "XBotRTPlugins");
    
    // define the XBotCore shared_memory for the RT plugins
    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();
    
    //
    _pluginHandler->load_plugins();
    
    //
    _pluginHandler->init_plugins(shared_memory, xbot_joint, xbot_ft, xbot_imu);
}

double XBot::XBotCore::get_time()
{
    return iit::ecat::get_time_ns() / 1e9;
}


int XBot::XBotCore::control_loop(void) 
{    
//     std::cout << "laurenzi" << std::endl;
    _iter++;
    _pluginHandler->run();
}



////////////////////////////////////
////////////////////////////////////
// SINGLE JOINT PRIVATE FUNCTIONS //
////////////////////////////////////
////////////////////////////////////

bool XBot::XBotCore::get_link_pos(int joint_id, double& link_pos)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        link_pos = motors[rid2Pos(joint_id)]->getRxPDO().link_pos * _conversion.link_pos;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_link_pos() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_motor_pos(int joint_id, double& motor_pos)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        motor_pos = motors[rid2Pos(joint_id)]->getRxPDO().motor_pos * _conversion.motor_pos;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_motor_pos() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_link_vel(int joint_id, double& link_vel)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        link_vel = motors[rid2Pos(joint_id)]->getRxPDO().link_vel * _conversion.link_vel;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_link_vel() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_motor_vel(int joint_id, double& motor_vel)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        motor_vel = motors[rid2Pos(joint_id)]->getRxPDO().motor_vel * _conversion.motor_vel;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_motor_vel() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_torque(int joint_id, double& torque)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        torque = motors[rid2Pos(joint_id)]->getRxPDO().torque * _conversion.torque;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_torque() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_temperature(int joint_id, double& temperature)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        temperature = motors[rid2Pos(joint_id)]->getRxPDO().temperature * _conversion.temperature;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_temperature() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_fault(int joint_id, double& fault)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        fault = motors[rid2Pos(joint_id)]->getRxPDO().fault * _conversion.fault;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_fault() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_rtt(int joint_id, double& rtt)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        rtt = motors[rid2Pos(joint_id)]->getRxPDO().rtt * _conversion.rtt;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_rtt() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_op_idx_ack(int joint_id, double& op_idx_ack)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        op_idx_ack = motors[rid2Pos(joint_id)]->getRxPDO().op_idx_ack * _conversion.op_idx_ack;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_op_idx_ack() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_aux(int joint_id, double& aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        aux = motors[rid2Pos(joint_id)]->getRxPDO().aux * _conversion.aux;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_gains(int joint_id, std::vector< double >& gain_vector)
{
    // resize the gain vector
    gain_vector.resize(5);
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        gain_vector[0] = motors[rid2Pos(joint_id)]->getTxPDO().gain_0 * _conversion.gains;
        gain_vector[1] = motors[rid2Pos(joint_id)]->getTxPDO().gain_1 * _conversion.gains;
        gain_vector[2] = motors[rid2Pos(joint_id)]->getTxPDO().gain_2 * _conversion.gains;
        gain_vector[3] = motors[rid2Pos(joint_id)]->getTxPDO().gain_3 * _conversion.gains;
        gain_vector[4] = motors[rid2Pos(joint_id)]->getTxPDO().gain_4 * _conversion.gains;
            
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_gains() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}




bool XBot::XBotCore::set_pos_ref(int joint_id, const double& pos_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        motors[rid2Pos(joint_id)]->set_posRef(pos_ref * _conversion.pos_ref);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_pos_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_vel_ref(int joint_id, const double& vel_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.vel_ref = vel_ref * _conversion.vel_ref;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_vel_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_tor_ref(int joint_id, const double& tor_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.tor_ref = tor_ref * _conversion.tor_ref;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_tor_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_gains(int joint_id, const std::vector<double>& gains)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        if(gains.size() == 5) {
            last_pdo_tx.gain_0 = gains[0] * _conversion.gains;
            last_pdo_tx.gain_1 = gains[1] * _conversion.gains;
            last_pdo_tx.gain_2 = gains[2] * _conversion.gains;
            last_pdo_tx.gain_3 = gains[3] * _conversion.gains;
            last_pdo_tx.gain_4 = gains[4] * _conversion.gains;
        }
        
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_gains() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}
 
bool XBot::XBotCore::set_fault_ack(int joint_id, const double& fault_ack)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.fault_ack = fault_ack * _conversion.fault_ack;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_fault_ack() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_ts(int joint_id, const double& ts)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data NOTE disabled: already done in on_writePDO
//         last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
//         last_pdo_tx.ts = ts;
//         motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_ts() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_op_idx_aux(int joint_id, const double& op_idx_aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.op_idx_aux = op_idx_aux * _conversion.op_idx_aux;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_op_idx_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_aux(int joint_id, const double& aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.aux = aux * _conversion.aux;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_ft(int ft_id, std::vector< double >& ft, int channels)
{
    // check if the joint requested exists
    if( fts.count(rid2Pos(ft_id)) ) {
        // get the data, resize the ft vector and copy only Fx,Fy,Fz and Tx,Ty,Tz
        iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft = fts[rid2Pos(ft_id)]->getRxPDO();
        ft.resize(channels);
        
        ft[0] = actual_pdo_rx_ft.force_X;
        ft[1] = actual_pdo_rx_ft.force_Y;
        ft[2] = actual_pdo_rx_ft.force_Z;
        ft[3] = actual_pdo_rx_ft.torque_X;
        ft[4] = actual_pdo_rx_ft.torque_Y;
        ft[5] = actual_pdo_rx_ft.torque_Z;
        
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_ft() on ft with ft_id : %d that does not exists\n", ft_id);
    return false;   
}


bool XBot::XBotCore::get_ft_fault(int ft_id, double& fault)
{
    // check if the joint requested exists
    if( fts.count(rid2Pos(ft_id)) ) {
        // get the data
        fault = fts[rid2Pos(ft_id)]->getRxPDO().fault;
        return true;
}
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_ft_fault() on ft with ft_id : %d that does not exists\n", ft_id);
    return false;  
}


bool XBot::XBotCore::get_ft_rtt(int ft_id, double& rtt)
{
    // check if the joint requested exists
    if( fts.count(rid2Pos(ft_id)) ) {
        // get the data
        rtt = fts[rid2Pos(ft_id)]->getRxPDO().rtt;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_ft_rtt() on ft with ft_id : %d that does not exists\n", ft_id);
    return false;   
}

bool XBot::XBotCore::get_imu(int imu_id, 
                             std::vector< double >& lin_acc, 
                             std::vector< double >& ang_vel, 
                             std::vector< double >& quaternion)
{
    // check if the joint requested exists
    if( imus.count(rid2Pos(imu_id)) ) {
        // get the data
        iit::ecat::advr::ImuEscPdoTypes::pdo_rx actual_pdo_rx_imu = imus[rid2Pos(imu_id)]->getRxPDO();
        
        quaternion.resize(4);
        quaternion[0] = actual_pdo_rx_imu.x_quat;
        quaternion[1] = actual_pdo_rx_imu.y_quat;
        quaternion[2] = actual_pdo_rx_imu.z_quat;
        quaternion[3] = actual_pdo_rx_imu.w_quat;
        
        lin_acc.resize(3);
        lin_acc[0] = actual_pdo_rx_imu.x_acc;
        lin_acc[1] = actual_pdo_rx_imu.y_acc;
        lin_acc[2] = actual_pdo_rx_imu.z_acc;
        
        ang_vel.resize(3);
        ang_vel[0] = actual_pdo_rx_imu.x_rate;
        ang_vel[1] = actual_pdo_rx_imu.y_rate;
        ang_vel[2] = actual_pdo_rx_imu.z_rate;
        
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_imu() on ft with imu_id : %d that does not exists\n", imu_id);
    return false;   
}

bool XBot::XBotCore::get_imu_fault(int imu_id, double& fault)
{
    // check if the joint requested exists
    if( imus.count(rid2Pos(imu_id)) ) {
        // get the data
        fault = imus[rid2Pos(imu_id)]->getRxPDO().fault;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_imu_fault() on ft with imu_id : %d that does not exists\n", imu_id);
    return false; 
}

bool XBot::XBotCore::get_imu_rtt(int imu_id, double& rtt)
{
    // check if the joint requested exists
    if( imus.count(rid2Pos(imu_id)) ) {
        // get the data
        rtt = imus[rid2Pos(imu_id)]->getRxPDO().rtt;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_imu_rtt() on ft with imu_id : %d that does not exists\n", imu_id);
    return false; 
}

bool XBot::XBotCore::grasp(int hand_id, double grasp_percentage)
{ 
    // check if the hand joint requested exists
    if( motors.count(rid2Pos(hand_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(hand_id)]->getTxPDO();
        // HACK 9.0 is assumed as maximum position range
        last_pdo_tx.pos_ref = (grasp_percentage * 9.0) * _conversion.pos_ref;
        motors[rid2Pos(hand_id)]->setTxPDO(last_pdo_tx);
//         DPRINTF("grasp %f - on joint with joint_id : %d - ecat_id : %d\n", last_pdo_tx.pos_ref, hand_id, rid2Pos(hand_id));
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to grasp() on joint with joint_id : %d that does not exists\n", hand_id);
    return false; 
}

double XBot::XBotCore::get_grasp_state(int hand_id)
{
    double link_pos = 0.0;
    double grasp_state = 0.0;
    // check if the hand joint requested exists
    if( motors.count(rid2Pos(hand_id)) ) {
        // get the data
        link_pos = motors[rid2Pos(hand_id)]->getRxPDO().link_pos * _conversion.link_pos;
        if( link_pos != 0.0) {
            // HACK 9.0 is assumed as maximum position range
            grasp_state = 9.0 / link_pos;
        }
        return grasp_state;
    }
    
    // we don't touch the value that you passed
//     DPRINTF("Trying to get_grasp_state() on joint with joint_id : %d that does not exists\n", hand_id);
    return -1;   
}



XBot::XBotCore::~XBotCore() {
    
    _pluginHandler->close();
    printf("Iteration: %d \n", _iter);
    printf("~XBotCore()\n");
}
