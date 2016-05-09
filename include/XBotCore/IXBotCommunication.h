/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __I_X_BOT_COMMUNICATION_H__
#define __I_X_BOT_COMMUNICATION_H__

#include <XBotCore/XBotCommunicationHandler.h>

namespace XBot
{
    class IXBotCommunication;
}

/**
 * @brief TBD
 * 
 */
class XBot::IXBotCommunication
{

public:   

    virtual bool init(std::shared_ptr<XBot::XBotCommunicationHandler>) = 0;       
    virtual ~IXBotCommunication() {};
};

#endif //__I_X_BOT_COMMUNICATION_H__
