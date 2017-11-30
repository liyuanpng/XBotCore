#ifndef __XBOT_OPT_H__
#define __XBOT_OPT_H__

#include <XBotInterface/RtLog.hpp>

namespace XBot {
 
    struct Options {
      
        /* General options */
        
        
        /* XBotCore options */
        
        int        xbotcore_period_us = 1000;
        bool          xbotcore_dummy_mode = false;
        bool          xbotcore_pluginhandler_catch_exceptions = true;
        bool          xbotcore_pluginhandler_print_stats = false;
        std::string   xbotcore_pluginhandler_plugin_set_name = "XBotRTPlugins";
        bool          xbotcore_load_transmission_plugins = true;
        
        
        /* CommHandler options */
        
        int comm_handler_period_us = 5000;
        bool   comm_handler_thread_enabled = true;
        bool   comm_handler_ros_publish_tf = true;
        
    };
    
}


#endif