#include <XBotCore/XBotCore.h>

XBotCore::XBotCore(const char* config_yaml) : XBotEcat(config_yaml)
{

    // XBotCore configuration
    const YAML::Node& root_cfg =  get_config_YAML_Node();
    const YAML::Node& x_bot_core_config = root_cfg["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();

}

XBotCoreModel XBotCore::get_robot_model(void)
{

    return model;
}

std::string XBotCore::get_urdf_path(void)
{
    
    return urdf_path;
}

std::string XBotCore::get_srdf_path(void)
{

    return srdf_path;
}

void XBotCore::parseJointMap(void)
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

void XBotCore::generateRobot(void)
{

}

void XBotCore::control_init(void) 
{
    // initialize the model
    if( !model.init( urdf_path, srdf_path ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
        return;
    }
    
    // parse the joint map YAML file and fill the joint_id from/to joint_name maps 
    parseJointMap();
    
    // TBD generate the chains data structures using the function from XBotCoreModel
    generateRobot();
    
    // call the plugin handler initialization
    if( !plugin_handler_init() ) {
        DPRINTF("ERROR: plugin handler initialization failed.\n");
        return;
    }
}


int XBotCore::control_loop(void) {
    
//     // start TEST
//     float float_aux = 0;
//     int16_t int16_t_aux = 0;
//     uint16_t uint16_t_aux = 0;
//     
//     DPRINTF("Board 123 :\n");
//     
//     get_link_pos(123, float_aux);
//     DPRINTF("get_link_pos(123, float_aux) = %d\n",float_aux);
//     
//     get_motor_pos(123, float_aux);
//     DPRINTF("get_motor_pos(123, float_aux) = %d\n",float_aux);
//     
//     get_link_vel(123, float_aux);
//     DPRINTF("get_link_vel(123, float_aux) = %d\n",float_aux);
//     
//     get_motor_vel(123, int16_t_aux);
//     DPRINTF("get_motor_vel(123, int16_t_aux) = %d\n",int16_t_aux);
//     
//     get_torque(123, int16_t_aux);
//     DPRINTF("get_torque(123, int16_t_aux) = %d\n",int16_t_aux);
//     
//     get_max_temperature(123, uint16_t_aux);
//     DPRINTF("get_max_temperature(123, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_fault(123, uint16_t_aux);
//     DPRINTF("get_fault(123, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_rtt(123, uint16_t_aux);
//     DPRINTF("get_rtt(123, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_op_idx_ack(123, uint16_t_aux);
//     DPRINTF("get_op_idx_ack(123, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_aux(123, float_aux);
//     DPRINTF("get_aux(123, float_aux) = %d\n",float_aux);
//     
//     DPRINTF("\n");
//     
//     DPRINTF("Board 2 :\n");
//     
//     get_link_pos(2, float_aux);
//     DPRINTF("get_link_pos(2, float_aux) = %d\n",float_aux);
//     
//     get_motor_pos(2, float_aux);
//     DPRINTF("get_motor_pos(2, float_aux) = %d\n",float_aux);
//     
//     get_link_vel(2, float_aux);
//     DPRINTF("get_link_vel(2, float_aux) = %d\n",float_aux);
//     
//     get_motor_vel(2, int16_t_aux);
//     DPRINTF("get_motor_vel(2, int16_t_aux) = %d\n",int16_t_aux);
//     
//     get_torque(2, int16_t_aux);
//     DPRINTF("get_torque(2, int16_t_aux) = %d\n",int16_t_aux);
//     
//     get_max_temperature(2, uint16_t_aux);
//     DPRINTF("get_max_temperature(2, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_fault(2, uint16_t_aux);
//     DPRINTF("get_fault(2, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_rtt(2, uint16_t_aux);
//     DPRINTF("get_rtt(2, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_op_idx_ack(2, uint16_t_aux);
//     DPRINTF("get_op_idx_ack(2, uint16_t_aux) = %d\n",uint16_t_aux);
//     
//     get_aux(2, float_aux);
//     DPRINTF("get_aux(2, float_aux) = %d\n",float_aux);
//     
//     DPRINTF("\n\n");
//     
//     // stop TEST
    
    
    
    // call the plugin handler loop
    return plugin_handler_loop();
}


bool XBotCore::get_link_pos(int joint_id, float& link_pos)
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

bool XBotCore::get_motor_pos(int joint_id, float& motor_pos)
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

bool XBotCore::get_link_vel(int joint_id, float& link_vel)
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

bool XBotCore::get_motor_vel(int joint_id, int16_t& motor_vel)
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

bool XBotCore::get_torque(int joint_id, int16_t& torque)
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

bool XBotCore::get_max_temperature(int joint_id, uint16_t& max_temperature)
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

bool XBotCore::get_fault(int joint_id, uint16_t& fault)
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

bool XBotCore::get_rtt(int joint_id, uint16_t& rtt)
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

bool XBotCore::get_op_idx_ack(int joint_id, uint16_t& op_idx_ack)
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

bool XBotCore::get_aux(int joint_id, float& aux)
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



XBotCore::~XBotCore() {
    
}
