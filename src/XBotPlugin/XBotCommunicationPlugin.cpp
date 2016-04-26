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
        for(int i=0; i< c.second.size(); i++) {
            xddps[c.second[i]] = std::make_shared<XDDP_pipe>();
            xddps[c.second[i]]->init(( std::string("rt_in_Motor_id_") + std::to_string(c.second[i])).c_str() );
        }
    }
    
    return true;
}

void XBot::XBotCommunicationPlugin::run(void)
{
    for( auto& p: xddps) {
        int n_bytes = p.second->xddp_read<iit::ecat::advr::McEscPdoTypes::pdo_tx>(pdo_tx);
        if(n_bytes > 0) {
            DPRINTF("read : %d bytes\n", n_bytes);
            DPRINTF("read pos_ref : %f\n", pdo_tx.pos_ref);
            std::fflush(stdout);
        }
    }
}

bool XBot::XBotCommunicationPlugin::close(void)
{
    return true;
}
