/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Murator (2016-, luca.muratore@iit.it)
*/

#ifndef __XBOTCORE_H__
#define __XBOTCORE_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/ecat/advr/esc.h>

#include <XBotCore/XBotCore_model.hpp>

/**
 * @brief TBD
 * 
 */
class XBotCore : public Ec_Thread_Boards_base
{
public:
    
    XBotCore(const char * config_yaml);
    virtual ~XBotCore();

    virtual int user_loop(void);
    
    uint16_t get_rtt(int slave_id);

protected:
    
    void parseSRDF();
    

private:
    
    virtual void init_preOP(void);
    virtual void init_OP(void);
    
    XBotCore_model model;
    std::string thread_name;
    
};

#endif //__XBOTCORE_H__
