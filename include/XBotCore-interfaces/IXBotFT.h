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


#ifndef __I_X_BOT_FT_H__
#define __I_X_BOT_FT_H__

#include <XBotInterface/RtLog.hpp>

namespace XBot
{
    class IXBotFT;
}

/**
 * @brief XBotCore F-T Force Torque Interface
 * 
 */
class XBot::IXBotFT
{

public:   
    
    virtual bool get_ft(int ft_id, std::vector<double>&  ft, int channels = 6) = 0;
        
    virtual bool get_ft_fault(int ft_id, double& fault) = 0;
    
    virtual bool get_ft_rtt(int ft_id, double& rtt) = 0;
    
    virtual ~IXBotFT() {
        if(Logger::GetVerbosityLevel() == Logger::Severity::LOW)
            std::cout << __func__ << std::endl;
    };
};

#endif //__I_X_BOT_FT_H__
