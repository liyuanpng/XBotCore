/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_CUBIC_TRAJECTORY_GENERATOR_H__
#define __X_BOT_CUBIC_TRAJECTORY_GENERATOR_H__

#include <XBotCore/XBotPlugin.hpp>

namespace XBot
{
    class XBotCubicTrajectoryGenerator;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotCubicTrajectoryGenerator : public XBot::XBotPlugin
{
    
public:   
    
    XBotCubicTrajectoryGenerator(std::string name,
                                 std::shared_ptr<XBot::IXBotModel> model, 
                                 std::shared_ptr<XBot::IXBotChain> chain,
                                 std::shared_ptr<XBot::IXBotRobot> robot,
                                 std::shared_ptr<XBot::XBotSharedMemory> memory);
    
    virtual bool init(void);
    virtual void run(void);
    virtual bool close(void);
    
private:
    
    bool compute_new_ref(std::map<int, float> pos_ref, float max_vel); 
    
    uint64_t start_time;

    std::map<int, float> a0, a1, a2, a3;
    
    bool init_param = true;
        
    std::map<int, float> robot_pos_ref;
    std::map<int, float> current_ref;
    

};

#endif //__X_BOT_CUBIC_TRAJECTORY_GENERATOR_H__
