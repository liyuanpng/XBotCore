#include <XBotCore/XBotPluginHandler.h>

#include <XBotPlugin/XBotTestPlugin.h>

XBot::XBotPluginHandler::XBotPluginHandler(const char* config_yaml): XBotCore(config_yaml)
{
    XBot::XBotCoreModel actual_model = get_robot_model();
    XBot::IXBotModel *actual_model_interface = dynamic_cast<XBot::IXBotModel *>(&actual_model);
    
    XBot::IXBotChain *actual_chain = dynamic_cast<XBot::IXBotChain *>(this);
    
    std::shared_ptr<XBot::XBotTestPlugin> test(new XBot::XBotTestPlugin(actual_model_interface, actual_chain));
    
    plugins.push_back(test);
}


bool XBot::XBotPluginHandler::plugin_handler_init(void)
{
    plugins_num = plugins.size();
    for(int i = 0; i < plugins_num; i++) {
        plugins[i]->init();
    }
    return true;
}


bool XBot::XBotPluginHandler::plugin_handler_loop(void)
{
    for(int i = 0; i < plugins_num; i++) {
        plugins[i]->run();
    }
    return true;
}

XBot::XBotPluginHandler::~XBotPluginHandler()
{

}