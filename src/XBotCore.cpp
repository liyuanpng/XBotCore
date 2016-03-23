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

XBotCoreModel XBot::XBotCore::get_robot_model(void)
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


void XBot::XBotCore::parseJointMap(void)
{
    // read the joint map config file -> we choose to separate it from the one used by XBotCore and ec_boards_iface
    joint_map_cfg = YAML::LoadFile(joint_map_config);
    const YAML::Node& joint_map = joint_map_cfg["joint_map"];

    // iterate over the node
    for(YAML::const_iterator it=joint_map.begin();it != joint_map.end();++it) {
        int tmp_rid = it->first.as<int>();
        std::string tmp_joint = it->second.as<std::string>();
        // fill the maps 
        rid2joint[tmp_rid] = tmp_joint;
        joint2rid[tmp_joint] = tmp_rid;

//         DPRINTF("rid2joint : rid -> %d ==> joint -> %s\n", tmp_rid, rid2joint[tmp_rid].c_str());
//         DPRINTF("joint2rid : joint -> %s ==> rid -> %d\n", tmp_joint.c_str(), joint2rid[tmp_joint]);
    }
}

void XBot::XBotCore::generateRobot(void)
{
    std::vector<std::string> actual_chain_names = model.get_chain_names();
    for( int i = 0; i < actual_chain_names.size(); i++) {
        std::vector<std::string> enabled_joints_name_aux;
        std::vector<int> enabled_joints_id_aux;
        if( model.get_enabled_joints_in_chain(actual_chain_names[i], enabled_joints_name_aux) ) {
            for( int j = 0; j < enabled_joints_name_aux.size(); j++ ) {
                if( joint2rid.count(enabled_joints_name_aux[j]) ) {
                    enabled_joints_id_aux.push_back(joint2rid.at(enabled_joints_name_aux[j]));
                }
            }
            robot[actual_chain_names[i]] = enabled_joints_id_aux;
        }
    }
}

void XBot::XBotCore::control_init(void) 
{
    // initialize the model
    if( !model.init( urdf_path, srdf_path ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
        return;
    }
    
    // parse the joint map YAML file and fill the joint_id from/to joint_name maps 
    parseJointMap();
    
    // generate the chains data structures using the function from XBotCoreModel
    generateRobot();
    
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


std::string XBot::XBotCore::rid2Joint(int rId)
{
    return rid2joint.find(rId) != rid2joint.end() ? rid2joint[rId] : ""; 
}

int XBot::XBotCore::joint2Rid(std::string joint_name)
{
    return joint2rid.find(joint_name) != joint2rid.end() ? joint2rid[joint_name] : 0; 
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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
            actual_joint_name = rid2joint.at(actual_chain_enabled_joints[i]);
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



XBot::XBotCore::~XBotCore() {
    
}
