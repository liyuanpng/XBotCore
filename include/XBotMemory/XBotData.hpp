/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_DATA_HPP__
#define __X_BOT_DATA_HPP__

#include <map>
#include <string>
#include <memory>

namespace XBot
{
    template <class T> class XBotData;
}

/**
 * @brief TBD
 * 
 */
template <class T>
class XBot::XBotData
{
    
public:   
    
    bool get(std::shared_ptr<T>& d) 
    { 
        if(is_new && data) {
            d = data; 
            is_new = false;
        }
    };
    
    bool set(const std::shared_ptr<T>& d)
    { 
        if(d) {
            data = d; 
            is_new = true;
        }
    };
    
    virtual ~XBotData() {};
    
private:
    
    std::shared_ptr<T> data;
    bool is_new = false;
};

#endif //__X_BOT_SHARED_MEMORY_HPP__
