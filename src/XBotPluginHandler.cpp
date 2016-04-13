#include <XBotCore/XBotPluginHandler.h>

#include <XBotPlugin/XBotTestPlugin.h>

XBot::XBotPluginHandler::XBotPluginHandler(const char* config_yaml): XBotCore(config_yaml)
{
    // TBD read the plugins to load from YAML
}

bool XBot::XBotPluginHandler::load_plugins() {

    XBot::XBotCoreModel model = get_robot_model();

    std::shared_ptr<XBot::IXBotModel> actual_model = std::make_shared<XBot::XBotCoreModel>(model);
    std::shared_ptr<XBot::IXBotChain> actual_chain(this);

    // TBD load dynamically the plugins
    std::shared_ptr<XBot::XBotTestPlugin> test(new XBot::XBotTestPlugin("test plugin",
                                                                        actual_model, 
                                                                        actual_chain));
    
    plugins.push_back(test);
    
    return true;
}


bool XBot::XBotPluginHandler::plugin_handler_init(void)
{
    // load the plugins
    if(!load_plugins()) {
        DPRINTF("ERROR: load_plugins() failed.");
        return false;
    }
    
    // iterate over the plugins and call the init()
    plugins_num = plugins.size();
    bool ret = true;
    for(int i = 0; i < plugins_num; i++) {
        if(!plugins[i]->init()) {
            DPRINTF("ERROR: plugin %s - init() failed\n", plugins[i]->name.c_str());
            ret = false;
        }
    }
    return ret;
}


bool XBot::XBotPluginHandler::plugin_handler_loop(void)
{
    for(int i = 0; i < plugins_num; i++) {
        plugins[i]->run();
    }
    return true;
}

bool XBot::XBotPluginHandler::plugin_handler_close(void)
{
    for(int i = 0; i < plugins_num; i++) {
        plugins[i]->run();
    }
    return true;
}


XBot::XBotPluginHandler::~XBotPluginHandler()
{

}