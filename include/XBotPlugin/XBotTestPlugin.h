/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_TEST_PLUGIN_H__
#define __X_BOT_TEST_PLUGIN_H__

#include <XBotCore/XBotPlugin.hpp>

namespace XBot
{
    class XBotTestPlugin;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotTestPlugin : public XBot::XBotPlugin
{
    
public:   
    
    XBotTestPlugin(std::string name,
                   std::shared_ptr<XBot::IXBotModel> model, 
                   std::shared_ptr<XBot::IXBotChain> chain);
    
    virtual bool init(void);
    virtual void run(void);
    virtual bool close(void);

};

#endif //__X_BOT_TEST_PLUGIN_H__
