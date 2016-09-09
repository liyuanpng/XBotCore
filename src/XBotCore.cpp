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

XBot::XBotCore::XBotCore(const char* config_yaml) : XBotEcat(config_yaml)
{

    // XBotCore configuration
    const YAML::Node& root_cfg =  get_config_YAML_Node();
    const YAML::Node& x_bot_core_config = root_cfg["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();

}

XBot::XBotCoreModel XBot::XBotCore::get_robot_model(void)
{
    return model;
}

std::string XBot::XBotCore::get_urdf_path(void)
{
    
    return urdf_path;
}

std::string XBot::XBotCore::get_srdf_path(void)
{

    return srdf_path;
}

std::vector< std::string > XBot::XBotCore::get_chain_names()
{
    return model.get_chain_names();
}

void XBot::XBotCore::control_init(void) 
{
    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
        return;
    }
    
    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    
    // call the plugin handler initialization
    if( !plugin_handler_init() ) {
        DPRINTF("ERROR: plugin handler initialization failed.\n");
        return;
    }
}


int XBot::XBotCore::control_loop(void) {
    
    // call the plugin handler loop
    return plugin_handler_loop();
}

///////////////////////////////
///////////////////////////////
// ROBOT PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool XBot::XBotCore::get_robot_link_pos(std::map< std::string, float >& link_pos)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_link_pos(std::map< int, float >& link_pos)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_motor_pos(std::map< std::string, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_motor_pos(std::map< int, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}


bool XBot::XBotCore::get_robot_link_vel(std::map< std::string, float >& link_vel)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_link_vel(std::map< int, float >& link_vel)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_motor_vel(std::map< std::string, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_motor_vel(std::map< int, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_torque(std::map< std::string, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_torque(std::map< int, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_max_temperature(std::map< std::string, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_max_temperature(std::map< int, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_fault(std::map< std::string, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_fault(std::map< int, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_rtt(std::map< std::string, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_rtt(std::map< int, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_op_idx_ack(std::map< std::string, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_op_idx_ack(std::map< int, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_aux(std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}

bool XBot::XBotCore::get_robot_aux(std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}




bool XBot::XBotCore::set_robot_pos_ref(const std::map< std::string, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_pos_ref(const std::map< int, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_vel_ref(const std::map< std::string, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_vel_ref(const std::map< int, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_tor_ref(const std::map< std::string, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_tor_ref(const std::map< int, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_gains(const std::map< std::string, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_gains(const std::map< int, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_fault_ack(const std::map< std::string, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_fault_ack(const std::map< int, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_ts(const std::map< std::string, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_ts(const std::map< int, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_op_idx_aux(const std::map< std::string, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_op_idx_aux(const std::map< int, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}


bool XBot::XBotCore::set_robot_aux(const std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}

bool XBot::XBotCore::set_robot_aux(const std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : robot) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}









///////////////////////////////
///////////////////////////////
// CHAIN PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool XBot::XBotCore::get_chain_link_pos(std::string chain_name, std::map< std::string, float>& link_pos)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            link_pos[actual_joint_name] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_link_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_pos() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_link_pos(std::string chain_name, std::map< int, float >& link_pos)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_link_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_pos() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_motor_pos(std::string chain_name, std::map< std::string, float >& motor_pos)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            motor_pos[actual_joint_name] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_motor_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_motor_pos(std::string chain_name, std::map< int, float >& motor_pos)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_motor_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_link_vel(std::string chain_name, std::map< std::string, float >& link_vel)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            link_vel[actual_joint_name] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_link_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_vel() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_link_vel(std::string chain_name, std::map< int, float >& link_vel)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_link_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_vel() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_motor_vel(std::string chain_name, std::map< std::string, int16_t >& motor_vel)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            motor_vel[actual_joint_name] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_motor_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_motor_vel(std::string chain_name, std::map< int, int16_t >& motor_vel)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_motor_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_torque(std::string chain_name, std::map< std::string, int16_t >& torque)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            torque[actual_joint_name] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_torque() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_torque() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_torque(std::string chain_name, std::map< int, int16_t >& torque)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            torque[actual_chain_enabled_joints[i]] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_torque() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_torque() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_max_temperature(std::string chain_name, std::map< std::string, uint16_t >& max_temperature)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            max_temperature[actual_joint_name] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_max_temperature() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_max_temperature(std::string chain_name, std::map< int, uint16_t >& max_temperature)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            max_temperature[actual_chain_enabled_joints[i]] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_max_temperature() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_fault(std::string chain_name, std::map< std::string, uint16_t >& fault)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            fault[actual_joint_name] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_fault() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_fault() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_fault(std::string chain_name, std::map< int, uint16_t >& fault)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            fault[actual_chain_enabled_joints[i]] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_fault() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_fault() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_rtt(std::string chain_name, std::map< std::string, uint16_t >& rtt)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            rtt[actual_joint_name] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_rtt() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_rtt() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_rtt(std::string chain_name, std::map< int, uint16_t >& rtt)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            rtt[actual_chain_enabled_joints[i]] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_rtt() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_rtt() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_op_idx_ack(std::string chain_name, std::map< std::string, uint16_t >& op_idx_ack)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            op_idx_ack[actual_joint_name] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_op_idx_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_op_idx_ack(std::string chain_name, std::map< int, uint16_t >& op_idx_ack)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            op_idx_ack[actual_chain_enabled_joints[i]] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_op_idx_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_aux(std::string chain_name, std::map< std::string, float >& aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            aux[actual_joint_name] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::get_chain_aux(std::string chain_name, std::map< int, float >& aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            aux[actual_chain_enabled_joints[i]] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}




bool XBot::XBotCore::set_chain_pos_ref(std::string chain_name, const std::map< std::string, float >& pos_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(pos_ref.count(actual_joint_name)) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_pos_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_pos_ref(std::string chain_name, const std::map< int, float >& pos_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(pos_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_pos_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_vel_ref(std::string chain_name, const std::map< std::string, int16_t >& vel_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(vel_ref.count(actual_joint_name)) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_vel_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_vel_ref(std::string chain_name, const std::map< int, int16_t >& vel_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(vel_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_vel_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_tor_ref(std::string chain_name, const std::map< std::string, int16_t >& tor_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(tor_ref.count(actual_joint_name)) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_tor_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_tor_ref(std::string chain_name, const std::map< int, int16_t >& tor_ref)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(tor_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_tor_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_gains(std::string chain_name, const std::map< std::string, std::vector<uint16_t> >& gains)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(gains.count(actual_joint_name)) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_chain_gains() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_gains(std::string chain_name, const std::map< int, std::vector<uint16_t> >& gains)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(gains.count(actual_chain_enabled_joints[i])) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_gains() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_gains() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_fault_ack(std::string chain_name, const std::map< std::string, int16_t >& fault_ack)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(fault_ack.count(actual_joint_name)) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_fault_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_fault_ack(std::string chain_name, const std::map< int, int16_t >& fault_ack)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(fault_ack.count(actual_chain_enabled_joints[i])) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_fault_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_ts(std::string chain_name, const std::map< std::string, uint16_t >& ts)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(ts.count(actual_joint_name)) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_ts() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_ts() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_ts(std::string chain_name, const std::map< int, uint16_t >& ts)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(ts.count(actual_chain_enabled_joints[i])) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_ts() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_ts() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_op_idx_aux(std::string chain_name, const std::map< std::string, uint16_t >& op_idx_aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(op_idx_aux.count(actual_joint_name)) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_op_idx_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_op_idx_aux(std::string chain_name, const std::map< int, uint16_t >& op_idx_aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(op_idx_aux.count(actual_chain_enabled_joints[i])) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_op_idx_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_aux(std::string chain_name, const std::map< std::string, float >& aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = model.rid2Joint(actual_chain_enabled_joints[i]);
            if(aux.count(actual_joint_name)) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name, chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}

bool XBot::XBotCore::set_chain_aux(std::string chain_name, const std::map< int, float >& aux)
{
    if( robot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(aux.count(actual_chain_enabled_joints[i])) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name);
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_aux() on chain %s, that does not exits in the robot\n", chain_name);
    return false;
}


////////////////////////////////////
////////////////////////////////////
// SINGLE JOINT PRIVATE FUNCTIONS //
////////////////////////////////////
////////////////////////////////////

bool XBot::XBotCore::get_link_pos(int joint_id, float& link_pos)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        link_pos = motors[rid2Pos(joint_id)]->getRxPDO().link_pos;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_link_pos() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_motor_pos(int joint_id, float& motor_pos)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        motor_pos = motors[rid2Pos(joint_id)]->getRxPDO().motor_pos;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_motor_pos() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_link_vel(int joint_id, float& link_vel)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        link_vel = motors[rid2Pos(joint_id)]->getRxPDO().link_vel;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_link_vel() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_motor_vel(int joint_id, int16_t& motor_vel)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        motor_vel = motors[rid2Pos(joint_id)]->getRxPDO().motor_vel;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_motor_vel() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}

bool XBot::XBotCore::get_torque(int joint_id, int16_t& torque)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        torque = motors[rid2Pos(joint_id)]->getRxPDO().torque;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_torque() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_max_temperature(int joint_id, uint16_t& max_temperature)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        max_temperature = motors[rid2Pos(joint_id)]->getRxPDO().max_temperature;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_max_temperature() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_fault(int joint_id, uint16_t& fault)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        fault = motors[rid2Pos(joint_id)]->getRxPDO().fault;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_fault() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_rtt(int joint_id, uint16_t& rtt)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        rtt = motors[rid2Pos(joint_id)]->getRxPDO().rtt;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_rtt() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_op_idx_ack(int joint_id, uint16_t& op_idx_ack)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        op_idx_ack = motors[rid2Pos(joint_id)]->getRxPDO().op_idx_ack;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_op_idx_ack() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;   
}

bool XBot::XBotCore::get_aux(int joint_id, float& aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // get the data
        aux = motors[rid2Pos(joint_id)]->getRxPDO().aux;
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false;  
}



bool XBot::XBotCore::set_pos_ref(int joint_id, const float& pos_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        motors[rid2Pos(joint_id)]->set_posRef(pos_ref);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_pos_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_vel_ref(int joint_id, const int16_t& vel_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.vel_ref = vel_ref;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_vel_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_tor_ref(int joint_id, const int16_t& tor_ref)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.tor_ref = tor_ref;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_tor_ref() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_gains(int joint_id, const std::vector<uint16_t>& gains)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        for(int i = 0; i < gains.size(); i++) {
            last_pdo_tx.gains[i] = gains[i];
        }
        
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_gains() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}
 
bool XBot::XBotCore::set_fault_ack(int joint_id, const int16_t& fault_ack)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.fault_ack = fault_ack;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_fault_ack() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_ts(int joint_id, const uint16_t& ts)
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

bool XBot::XBotCore::set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.op_idx_aux = op_idx_aux;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_op_idx_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::set_aux(int joint_id, const float& aux)
{
    
    // check if the joint requested exists
    if( motors.count(rid2Pos(joint_id)) ) {
        // set the data
        last_pdo_tx = motors[rid2Pos(joint_id)]->getTxPDO();
        last_pdo_tx.aux = aux;
        motors[rid2Pos(joint_id)]->setTxPDO(last_pdo_tx);
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to set_aux() on joint with joint_id : %d that does not exists\n", joint_id);
    return false; 
}

bool XBot::XBotCore::get_ft(int ft_id, std::vector< float >& ft, int channels)
{
    // check if the joint requested exists
    if( fts.count(rid2Pos(ft_id)) ) {
        // get the data, resize the ft vector and copy only Fx,Fy,Fz and Tx,Ty,Tz
        iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft = fts[rid2Pos(ft_id)]->getRxPDO();
        ft.resize(channels);
        std::memcpy(ft.data(), &(actual_pdo_rx_ft.force_X), channels*sizeof(float));
        
        return true;
    }
    
    // we don't touch the value that you passed
    DPRINTF("Trying to get_ft() on ft with ft_id : %d that does not exists\n", ft_id);
    return false;   
}


bool XBot::XBotCore::get_ft_fault(int ft_id, uint16_t& fault)
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


bool XBot::XBotCore::get_ft_rtt(int ft_id, uint16_t& rtt)
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



XBot::XBotCore::~XBotCore() {
    printf("~XBotCore()\n");
}
