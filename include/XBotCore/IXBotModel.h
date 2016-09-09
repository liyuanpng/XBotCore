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


#ifndef __I_X_BOT_MODEL_H__
#define __I_X_BOT_MODEL_H__

#include <map>
#include <string>
#include <vector>

namespace XBot
{
    class IXBotModel;
}

/**
 * @brief XBotCore Model Interface
 * 
 */
class XBot::IXBotModel
{
    
public:   

    virtual std::map<std::string, std::vector<int> >  get_robot() = 0;
    virtual std::map<std::string, int>  get_ft_sensors() = 0;
    
    virtual std::string rid2Joint(int rId) = 0;
    virtual int joint2Rid(std::string joint_name) = 0;
    

    virtual ~IXBotModel() {
        printf("~IXBotModel\n");
    };
};

#endif //__I_X_BOT_MODEL_H__
