/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_YARP_PLUGIN_H__
#define __X_BOT_YARP_PLUGIN_H__

#include <XBotCore/XBotPlugin.hpp>

namespace XBot
{
    class XBotYarpPlugin;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotYarpPlugin : public XBot::XBotPlugin
{
    
public:   
    
    virtual bool init(void);
    
    virtual void run(void);
    
    virtual bool close(void);

};

#endif //__X_BOT_YARP_PLUGIN_H__
