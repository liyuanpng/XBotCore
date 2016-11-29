/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>       
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/


#ifndef __X_BOT_PLUGIN_HPP__
#define __X_BOT_PLUGIN_HPP__

#include <memory>

#include <XBotCore/XBotCore.h>
#include <XBotCore/IXBotModel.h>
#include <XBotCore/IXBotRobot.h>

namespace XBot
{
    class XBotPlugin;
}

/**
 * @brief XBotCore Plugin Interface.
 * 
 */
class XBot::XBotPlugin
{
    
public:   
    
    XBotPlugin();
    
    virtual ~XBotPlugin();

    virtual bool init(std::string name,
                      std::shared_ptr<XBot::IXBotModel> model, 
                      std::shared_ptr<XBot::IXBotChain> chain,
                      std::shared_ptr<XBot::IXBotRobot> robot,
                      std::shared_ptr<XBot::IXBotFT> ft) = 0;
                      
    virtual void run() = 0;
    
    virtual bool close() = 0;
    

    std::string name;
    
protected:
    
    std::shared_ptr<XBot::IXBotModel> model;
    std::shared_ptr<XBot::IXBotChain> chain;
    std::shared_ptr<XBot::IXBotRobot> robot;
    std::shared_ptr<XBot::IXBotFT> ft;


};

#endif //__X_BOT_PLUGIN_HPP__
