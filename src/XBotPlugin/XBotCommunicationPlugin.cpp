#include <XBotPlugin/XBotCommunicationPlugin.h>

#define MAX_ALLOWED_TEMPERATURE 70

XBot::XBotCommunicationPlugin::XBotCommunicationPlugin( std::string name,
                                                        std::shared_ptr<XBot::IXBotModel> model, 
                                                        std::shared_ptr<XBot::IXBotChain> chain,
                                                        std::shared_ptr<XBot::IXBotRobot> robot,
                                                        std::shared_ptr<XBot::IXBotFT> ft) :
                                                        XBotPlugin(name, model, chain, robot, ft)
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
            pos_ref_map[p.first] = pdo_tx.pos_ref;
        }
    }
    if(!pos_ref_map.empty()) {
//         for ( auto& b: pos_ref_map) {
//             if(b.first == 21) {
//                 DPRINTF("board %d : ", b.first);
//                 DPRINTF("setting pos_ref : %f\n", b.second);
//             }
//         }
        
        robot->set_robot_pos_ref(pos_ref_map); // TBD do a set for each settable field
    }
    
    // check the temperature
    robot->get_robot_max_temperature(temperature);
    for( auto& temp: temperature) {
        if( temp.second > MAX_ALLOWED_TEMPERATURE ) {
            DPRINTF("WARN: TEMPERATURE TOO HIGH ON JOINT %d -> %u C\n", temp.first, (unsigned int) temp.second);
        }
    }
}

bool XBot::XBotCommunicationPlugin::close(void)
{
    return true;
}

XBot::XBotCommunicationPlugin::~XBotCommunicationPlugin()
{
    printf("~XBotCommunicationPlugin()\n");
}

