#include <XBotPlugin/XBotTestPlugin.h>

XBot::XBotTestPlugin::XBotTestPlugin(XBot::IXBotModel* model, XBot::IXBotChain* chain): XBotPlugin(model, chain)
{

}


bool XBot::XBotTestPlugin::init(void)
{
    DPRINTF("init()\n");
}

void XBot::XBotTestPlugin::run(void)
{
    std::map<std::string, uint16_t> l_arm_rtt;
    chain->get_chain_rtt("left_arm", l_arm_rtt);

    for( auto& j : l_arm_rtt) {
        DPRINTF("left_arm Joint : %d - RTT : %d\n",  model->joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<int, uint16_t> l_hand_j_rtt;
    chain->get_chain_rtt("left_hand", l_hand_j_rtt);

    for( auto& j : l_hand_j_rtt) {
        DPRINTF("left_hand Joint : %d - RTT : %d\n", j.first, j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<std::string, uint16_t> l_arm_temperature;
    chain->get_chain_max_temperature("left_arm", l_arm_temperature);

    for( auto& j : l_arm_temperature) {
        DPRINTF("Joint : %d - TEMPERATURE : %d\n",  model->joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    std::map<std::string, int16_t> l_arm_torque;
    chain->get_chain_torque("left_arm", l_arm_torque);

    for( auto& j : l_arm_torque) {
        DPRINTF("Joint : %d - TORQUE : %d\n", model->joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
}

bool XBot::XBotTestPlugin::close(void)
{

}
