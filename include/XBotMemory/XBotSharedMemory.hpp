/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_SHARED_MEMORY_HPP__
#define __X_BOT_SHARED_MEMORY_HPP__

#include <map>
#include <string>
#include <memory>

#include <XBotMemory/XBotData.hpp>
#include <boost/any.hpp>

namespace XBot
{
    class XBotSharedMemory;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotSharedMemory
{
    
public:   
    
    bool set_plugin_memory(std::string plugin_name, const std::shared_ptr< XBot::XBotData<boost::any> >& p_m)
    {
        plugins_memory[plugin_name] = p_m;
        return true;
    }
        
    bool get_plugin_memory(std::string plugin_name, std::shared_ptr< XBot::XBotData<boost::any> >& p_m)
    {
        if(plugins_memory.count(plugin_name) ) {
            p_m = plugins_memory.at(plugin_name);
            return true;
        }
        else {
            DPRINTF("ERROR: no memory for the plugin: %s - get_plugin_memory() failed\n", plugin_name);
            return false;
        }
    }
    
    virtual ~XBotSharedMemory() {};
private:
        
    std::map<std::string, std::shared_ptr< XBot::XBotData<boost::any> > > plugins_memory; 
    

};

#endif //__X_BOT_SHARED_MEMORY_HPP__
