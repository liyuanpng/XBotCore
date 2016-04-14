#include <XBotPlugin/XBotTestPlugin.h>

XBot::XBotTestPlugin::XBotTestPlugin(std::string name,
                                     std::shared_ptr<XBot::IXBotModel> model, 
                                     std::shared_ptr<XBot::IXBotChain> chain) : 
                                     XBotPlugin(name, model, chain)
{

}


bool XBot::XBotTestPlugin::init(void)
{
    DPRINTF("XBotTestPlugin init()\n");
    l_arm_pos_ref[123] = 0;
    
    return true;
}

void XBot::XBotTestPlugin::run(void)
{
    std::map<std::string, uint16_t> l_arm_rtt;
    chain->get_chain_rtt("left_arm", l_arm_rtt);

    for( auto& j : l_arm_rtt) {
        std::string j_name = j.first;
        int rtt = j.second;
        int rid = model->joint2Rid(j_name);
        //DPRINTF("left_arm Joint : %d - RTT : %d\n", rid, rtt); // NOTE avoid printing std::string with XENOMAI printf 
    }

    
//     std::map<int, uint16_t> l_hand_j_rtt;
//     chain->get_chain_rtt("left_hand", l_hand_j_rtt);
// 
//     for( auto& j : l_hand_j_rtt) {
//         DPRINTF("left_hand Joint : %d - RTT : %d\n", j.first, j.second); // NOTE avoid printing std::string with XENOMAI printf 
//     }
//     
//     std::map<std::string, uint16_t> l_arm_temperature;
//     chain->get_chain_max_temperature("left_arm", l_arm_temperature);
// 
//     for( auto& j : l_arm_temperature) {
//         DPRINTF("Joint : %d - TEMPERATURE : %d\n",  model->joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
//     }
//     
//     std::map<std::string, int16_t> l_arm_torque;
//     chain->get_chain_torque("left_arm", l_arm_torque);
// 
//     for( auto& j : l_arm_torque) {
//         DPRINTF("Joint : %d - TORQUE : %d\n", model->joint2Rid(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
//     }
    
    
    
    l_arm_pos_ref[123] += 0.1;
    
    chain->set_chain_pos_ref("left_arm", l_arm_pos_ref);
    
    std::map<int, int16_t> l_arm_vel_ref;
    l_arm_vel_ref[123] = 4;
    
    chain->set_chain_vel_ref("left_arm", l_arm_vel_ref);
    
    std::map<int, std::vector<uint16_t>> l_arm_gains;
    std::vector<uint16_t> g(5);
    g[0] = 210;
    g[1] = 211;
    g[2] = 212;
    g[3] = 213;
    g[4] = 214;
    l_arm_gains[123] = g;
    
    chain->set_chain_gains("left_arm", l_arm_gains);

    
    
}

bool XBot::XBotTestPlugin::close(void)
{
    DPRINTF("XBotTestPlugin close()\n");
}
