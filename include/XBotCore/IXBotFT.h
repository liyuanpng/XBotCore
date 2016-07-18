/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __I_X_BOT_FT_H__
#define __I_X_BOT_FT_H__

namespace XBot
{
    class IXBotFT;
}

/**
 * @brief TBD
 * 
 */
class XBot::IXBotFT
{

public:   
    
    virtual bool get_ft(int ft_id, std::vector<float>&  ft, int channels = 6) = 0;
        
    virtual bool get_ft_fault(int ft_id, uint16_t& fault) = 0;
    
    virtual bool get_ft_rtt(int ft_id, uint16_t& rtt) = 0;
    
    virtual ~IXBotFT() {
        printf("~IXBotFT()\n");
    };
};

#endif //__I_X_BOT_FT_H__
