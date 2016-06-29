#include <XBotPlugin/XBotCommunicationPlugin.h>

XBot::XBotCommunicationPlugin::XBotCommunicationPlugin( std::string name,
                                                        std::shared_ptr<XBot::IXBotModel> model, 
                                                        std::shared_ptr<XBot::IXBotChain> chain,
                                                        std::shared_ptr<XBot::IXBotRobot> robot,
                                                        std::shared_ptr<XBot::XBotSharedMemory> memory) :
                                                        XBotPlugin(name, model, chain, robot, memory)
{

}

bool XBot::XBotCommunicationPlugin::init(void)
{
    for(auto& c : model->get_robot()) {
        for(int i=0; i< c.second.size(); i++) { // TBD check if the motor exist
            xddps[c.second[i]] = std::make_shared<XDDP_pipe>();
            xddps[c.second[i]]->init(( std::string("rt_in_Motor_id_") + std::to_string(c.second[i])).c_str() );
        }
    }
    
    return true;
}

void XBot::XBotCommunicationPlugin::run(void)
{
    std::map<int, float> pos_ref_map;
    for( auto& p: xddps) {
        int n_bytes = p.second->xddp_read<iit::ecat::advr::McEscPdoTypes::pdo_tx>(pdo_tx);
        if(n_bytes > 0) {
//             DPRINTF("board %d : ", p.first);
//             DPRINTF("reading pos_ref : %f\n", pdo_tx.pos_ref);
            pos_ref_map[p.first] = pdo_tx.pos_ref;  
        }
    }
    if(!pos_ref_map.empty()) {
//         for ( auto& b: pos_ref_map) {
//             DPRINTF("board %d : ", b.first);
//             DPRINTF("setting pos_ref : %f\n", b.second);
//         }

        robot->set_robot_pos_ref(pos_ref_map); // TBD do a set for each settable field
    }
}

bool XBot::XBotCommunicationPlugin::close(void)
{
    return true;
}
