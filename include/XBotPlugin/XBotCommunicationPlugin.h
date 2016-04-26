/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_COMMUNICATION_PLUGIN_H__
#define __X_BOT_COMMUNICATION_PLUGIN_H__

#include <XBotCore/XBotPlugin.hpp>

#include <iit/advr/pipes.h>
#include <iit/ecat/advr/esc.h>

#include <memory>

namespace XBot
{
    class XBotCommunicationPlugin;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotCommunicationPlugin : public XBot::XBotPlugin
{
private:
    std::map<int,std::shared_ptr<XDDP_pipe>> xddps;
    iit::ecat::advr::McEscPdoTypes::pdo_tx pdo_tx;
    
public:  
    
    XBotCommunicationPlugin(std::string name,
                            std::shared_ptr<XBot::IXBotModel> model, 
                            std::shared_ptr<XBot::IXBotChain> chain,
                            std::shared_ptr<XBot::IXBotRobot> robot,
                            std::shared_ptr<XBot::XBotSharedMemory> memory);

    virtual bool init(void);
    
    virtual void run(void);
    
    virtual bool close(void);
    

};

#endif //__X_BOT_COMMUNICATION_PLUGIN_H__
