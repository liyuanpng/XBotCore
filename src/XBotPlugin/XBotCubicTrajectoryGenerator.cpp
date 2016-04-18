#include <XBotPlugin/XBotCubicTrajectoryGenerator.h>

XBot::XBotCubicTrajectoryGenerator::XBotCubicTrajectoryGenerator(std::string name,
                                                                 std::shared_ptr<XBot::IXBotModel> model, 
                                                                 std::shared_ptr<XBot::IXBotChain> chain) : 
                                                                 XBotPlugin(name, model, chain)
{

}


bool XBot::XBotCubicTrajectoryGenerator::init(void)
{
    DPRINTF("XBotCubicTrajectoryGenerator init()\n");
    
    // l_arm_pos_ref init
    std::map<int, float> l_arm_link_pos;
    chain->get_chain_link_pos("left_arm", l_arm_link_pos);
    for( auto& j : l_arm_link_pos) {
        DPRINTF("l_arm_link_pos Joint : %d - link pos : %f\n", j.first, j.second);
//         l_arm_pos_ref[j.first] = j.second;
    }
    // start time
    start_time = iit::ecat::get_time_ns();
    
    // NOTE test
    l_arm_pos_ref[25] = -0.3;
    l_arm_pos_ref[26] = -0.2;
    l_arm_pos_ref[27] = -0.4;
    
    return true;
}

bool XBot::XBotCubicTrajectoryGenerator::compute_new_ref(std::map<int, float> pos_ref, float max_vel)
{
    uint64_t tNow = iit::ecat::get_time_ns();
    float dt = ( tNow - start_time ) / 1e9;
    
    if (init_param) {
        // get the link pos
        std::map<int, float> l_arm_link_pos;
        chain->get_chain_link_pos("left_arm", l_arm_link_pos);

        for( auto& j : l_arm_pos_ref) {
            float t_f = std::abs((pos_ref[j.first] - l_arm_link_pos[j.first])) / max_vel;
            a0[j.first] =  l_arm_link_pos.at(j.first);
            a1[j.first] =  0; // NOTE standard case
            a2[j.first] =  3 * (pos_ref.at(j.first) - l_arm_link_pos.at(j.first)) / (t_f*t_f);
            a3[j.first] = -2 * (pos_ref.at(j.first) - l_arm_link_pos.at(j.first)) / (t_f*t_f*t_f);
            // NOTE print out params
            DPRINTF("%d -> t_f : %f a0 : %f a1 : %f a2 : %f a3 : %f \n",j.first, t_f, a0[j.first],a1[j.first],a2[j.first],a3[j.first]);
        }
        
        init_param = false;
    }

    for( auto& j : l_arm_pos_ref) {
        if(std::abs(current_ref[j.first] - pos_ref[j.first]) > 0.005)  {
            current_ref[j.first] = a3.at(j.first)*dt*dt*dt + a2.at(j.first)*dt*dt + a1.at(j.first)*dt + a0.at(j.first);
        }
    }

    return true;
}


void XBot::XBotCubicTrajectoryGenerator::run(void)
{  
    // communication between plugin in order to have a new reference

    
    compute_new_ref(l_arm_pos_ref, 0.2);
    //     chain->set_chain_aux("left_arm", current_ref);
    chain->set_chain_pos_ref("left_arm", current_ref);

}

bool XBot::XBotCubicTrajectoryGenerator::close(void)
{
    DPRINTF("XBotCubicTrajectoryGenerator close()\n");
}
