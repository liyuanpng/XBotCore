/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
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
 * @brief TBD
 * 
 */
class XBot::IXBotModel
{
    
public:   

    virtual std::map< std::string, std::vector<int> >  get_robot() = 0;
    virtual std::map<std::string, int>  get_ft_sensors() = 0;
    virtual std::map<std::string, int>  get_imu_sensors() = 0;
    
    virtual std::string rid2Joint(int rId) = 0;
    virtual int joint2Rid(std::string joint_name) = 0;
    

    virtual ~IXBotModel() {};
};

#endif //__I_X_BOT_MODEL_H__
