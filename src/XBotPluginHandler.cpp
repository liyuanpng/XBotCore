#include <XBotCore/XBotPluginHandler.h>

XBot::XBotPluginHandler::XBotPluginHandler(const char* config_yaml): XBotCore(config_yaml)
{

}


bool XBot::XBotPluginHandler::plugin_handler_init(void)
{
    
    return true;
}


bool XBot::XBotPluginHandler::plugin_handler_loop(void)
{
    std::map<std::string, uint16_t> l_arm_rtt;
    get_chain_rtt("left_arm", l_arm_rtt);

    for( auto& j : l_arm_rtt) {
        DPRINTF("left_arm Joint : %d - RTT : %d\n", joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<int, uint16_t> l_hand_j_rtt;
    get_chain_rtt("left_hand", l_hand_j_rtt);

    for( auto& j : l_hand_j_rtt) {
        DPRINTF("left_hand Joint : %d - RTT : %d\n", j.first, j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<std::string, uint16_t> l_arm_temperature;
    get_chain_max_temperature("left_arm", l_arm_temperature);

    for( auto& j : l_arm_temperature) {
        DPRINTF("Joint : %d - TEMPERATURE : %d\n", joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<std::string, int16_t> l_arm_torque;
    get_chain_torque("left_arm", l_arm_torque);

    for( auto& j : l_arm_torque) {
        DPRINTF("Joint : %d - TORQUE : %d\n", joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    return true;
}

XBot::XBotPluginHandler::~XBotPluginHandler()
{

}