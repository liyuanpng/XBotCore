#include <XBotCore/XBotPluginHandler.h>

#include <XBotPlugin/XBotCommunicationPlugin.h>

XBot::XBotPluginHandler::XBotPluginHandler(const char* config_yaml): XBotCore(config_yaml)
{
    // TBD read the plugins to load from YAML
}

bool XBot::XBotPluginHandler::load_plugins() {

    XBot::XBotCoreModel model = get_robot_model();

    std::shared_ptr<XBot::IXBotModel> actual_model = std::make_shared<XBot::XBotCoreModel>(model);
    std::shared_ptr<XBot::IXBotChain> actual_chain(this);
    std::shared_ptr<XBot::IXBotRobot> actual_robot(this);
    std::shared_ptr<XBot::IXBotFT> actual_ft(this);
    
    // TBD load dynamically the plugins
    std::shared_ptr<XBot::XBotCommunicationPlugin> communication_plugin(new XBot::XBotCommunicationPlugin( "communication plugin",
                                                                                                            actual_model, 
                                                                                                            actual_chain,
                                                                                                            actual_robot,
                                                                                                            actual_ft));
    plugins.push_back(communication_plugin);
    
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
    std::vector<float> plugin_execution_time(plugins_num); // TBD circular array and write to file in the plugin_handler_close
    for(int i = 0; i < plugins_num; i++) {
        float plugin_start_time = (iit::ecat::get_time_ns() / 10e3); //microsec
        plugins[i]->run();
        plugin_execution_time[i] = (iit::ecat::get_time_ns() / 10e3) - plugin_start_time; //microsec
//         DPRINTF("Plugin %d - %s : execution_time = %f microsec\n", i, plugins[i]->name.c_str(), plugin_execution_time[i]);
    }
    return true;
}

bool XBot::XBotPluginHandler::plugin_handler_close(void)
{
    bool ret = true;
    for(int i = 0; i < plugins_num; i++) {
        if(!plugins[i]->close()) {
            DPRINTF("ERROR: plugin %s - close() failed\n", plugins[i]->name.c_str());
            ret = false;
        }
    }
    return ret;
}

XBot::XBotPluginHandler::~XBotPluginHandler()
{
    printf("~XBotPluginHandler()\n");
}