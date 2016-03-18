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
    
    return true;
}

XBotPluginHandler::~XBotPluginHandler()
{

}