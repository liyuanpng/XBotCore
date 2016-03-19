#include <XBotCore/XBotPluginHandler.h>

XBotPluginHandler::XBotPluginHandler(const char* config_yaml): XBotCore(config_yaml)
{

}


bool XBotPluginHandler::plugin_handler_init(void)
{
    
    return true;
}


bool XBotPluginHandler::plugin_handler_loop(void)
{
    std::map<std::string, uint16_t> l_arm_rtt;
    get_chain_rtt("left_arm", l_arm_rtt);

    for( auto& j : l_arm_rtt) {
        DPRINTF("Joint : %d - RTT : %d\n",joint2rid.at(j.first), j.second); // NOTE avoid printing std::string with XENOMAI printf 
    }
    
    return true;
}

XBotPluginHandler::~XBotPluginHandler()
{

}