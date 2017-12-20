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

#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotModel.h>
#include <XBotCore-interfaces/IXBotFT.h>
#include <XBotCore-interfaces/IXBotIMU.h>
#include <XBotCore-interfaces/IXBotHand.h>
#include <XBotCore-interfaces/XBotSharedMemory.h>
#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/XBotHandle.h>
#include <XCM/XBotPluginStatus.h>
#include <XBotCore/HALInterface.h>

#include <map>
#include <vector>
#include <memory>
#include <string>

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

    virtual bool init(XBot::Handle::Ptr handle,
                      std::string name,
                      std::shared_ptr<PluginStatus> cstatus,
                      std::shared_ptr<HALInterface> halInterface,
                      std::shared_ptr<XBot::IXBotModel> model ) = 0;
                      
    virtual void run(double time, double period) = 0;
    
    virtual bool close() = 0;
    

    std::string name;
    
protected:
    
    std::shared_ptr<HALInterface> get_xbotcore_halInterface();
    std::shared_ptr<XBot::IXBotJoint> get_xbotcore_joint();
    std::shared_ptr<XBot::IXBotModel> get_xbotcore_model();
    std::shared_ptr<XBot::IXBotFT>    get_xbotcore_ft();
    std::shared_ptr<XBot::IXBotIMU>   get_xbotcore_imu();
    std::shared_ptr<XBot::IXBotHand>   get_xbotcore_hand();
    
    void set_xbotcore_halInterface(std::shared_ptr<HALInterface> halInterface);
    void set_xbotcore_joint(std::shared_ptr<XBot::IXBotJoint> joint);
    void set_xbotcore_model(std::shared_ptr<XBot::IXBotModel> model);
    void set_xbotcore_ft(std::shared_ptr<XBot::IXBotFT> ft);
    void set_xbotcore_imu(std::shared_ptr<XBot::IXBotIMU> imu);
    void set_xbotcore_hand(std::shared_ptr<XBot::IXBotHand> hand);
    
    
private:
    
    std::shared_ptr<XBot::IXBotJoint> _joint;
    std::shared_ptr<XBot::IXBotModel> _model;
    std::shared_ptr<XBot::IXBotFT>    _ft;
    std::shared_ptr<XBot::IXBotIMU>   _imu;
    std::shared_ptr<XBot::IXBotHand>   _hand;
    std::shared_ptr<HALInterface> _halInterface;

};

#endif //__X_BOT_PLUGIN_HPP__
