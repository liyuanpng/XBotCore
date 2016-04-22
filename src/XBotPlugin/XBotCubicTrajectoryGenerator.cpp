#include <XBotPlugin/XBotCubicTrajectoryGenerator.h>

#include <XBotMemory/XBotData.hpp>

XBot::XBotCubicTrajectoryGenerator::XBotCubicTrajectoryGenerator(std::string name,
                                                                 std::shared_ptr<XBot::IXBotModel> model, 
                                                                 std::shared_ptr<XBot::IXBotChain> chain,
                                                                 std::shared_ptr<XBot::IXBotRobot> robot,
                                                                 std::shared_ptr<XBot::XBotSharedMemory> memory) : 
                                                                 XBotPlugin(name, model, chain, robot, memory)
{

}


bool XBot::XBotCubicTrajectoryGenerator::init(void)
{
    DPRINTF("XBotCubicTrajectoryGenerator init()\n");
    
    // l_arm_pos_ref init
    std::map<int, float> robot_link_pos;
    robot->get_robot_link_pos(robot_link_pos);
    for( auto& j : robot_link_pos) {
        DPRINTF("robot Joint : %d - link pos : %f\n", j.first, j.second);
//         l_arm_pos_ref[j.first] = j.second;
    }
    // start time
    start_time = iit::ecat::get_time_ns();
    
    // NOTE test
    robot_pos_ref[25] = -0.3;
    robot_pos_ref[26] = -0.2;
    robot_pos_ref[27] = -0.4;
    robot_pos_ref[29] = 3.0;
    
    // declare the XBotData shared pointer
    auto p = std::make_shared<XBot::XBotData<boost::any>>();
    // store the data
//     float to_store = 0.5;
    std::map<int,float> to_store;
    to_store[29] = 2.0;
    
    auto data = std::make_shared<boost::any>(to_store);
    if(p->set(data)) {
         std::map<int,float> d = boost::any_cast<std::map<int,float>>(*data);
        DPRINTF("---------------- %f\n", d.at(29));
    }
    // store the plugin memory
    memory->set_plugin_memory("test plugin", p);


    
    
    
    return true;
}

bool XBot::XBotCubicTrajectoryGenerator::compute_new_ref(std::map<int, float> pos_ref, float max_vel)
{
    uint64_t tNow = iit::ecat::get_time_ns();
    float dt = ( tNow - start_time ) / 1e9;
    
    if (init_param) {
        // get the link pos
        std::map<int, float> robot_link_pos;
        robot->get_robot_link_pos(robot_link_pos);

        for( auto& j : pos_ref) {
            // init the cubic params
            float t_f = std::abs((pos_ref.at(j.first) - robot_link_pos.at(j.first))) / max_vel;
            a0[j.first] =  robot_link_pos.at(j.first);
            a1[j.first] =  0; // NOTE standard case
            a2[j.first] =  3 * (pos_ref.at(j.first) - robot_link_pos.at(j.first)) / (t_f*t_f);
            a3[j.first] = -2 * (pos_ref.at(j.first) - robot_link_pos.at(j.first)) / (t_f*t_f*t_f);
            
            //init current reference
            current_ref[j.first] = a3.at(j.first)*dt*dt*dt + a2.at(j.first)*dt*dt + a1.at(j.first)*dt + a0.at(j.first);
            
            // NOTE print out params
            DPRINTF("%d -> t_f : %f a0 : %f a1 : %f a2 : %f a3 : %f \n",j.first, t_f, a0[j.first],a1[j.first],a2[j.first],a3[j.first]);
        }
        
        init_param = false;
    }

    for( auto& j : pos_ref) {
        if(std::abs(current_ref.at(j.first) - pos_ref.at(j.first)) > 0.005)  {
            current_ref[j.first] = a3.at(j.first)*dt*dt*dt + a2.at(j.first)*dt*dt + a1.at(j.first)*dt + a0.at(j.first);
        }
    }

    return true;
}


void XBot::XBotCubicTrajectoryGenerator::run(void)
{  
    // communication between plugin in order to have a new reference

    // declare the XBotData shared pointer
    auto p = std::make_shared<XBot::XBotData<boost::any>>();
    // retrieve the plugin memory
    memory->get_plugin_memory("test plugin", p);
    // retrieve the data stored if any new
    auto data = std::make_shared<boost::any>();
    if(p->get(data)) {    
        std::map<int,float> d = boost::any_cast<std::map<int,float>>(*data);
        DPRINTF("**************** %f\n", d.at(29));
    }

    
    compute_new_ref(robot_pos_ref, 0.2);
    robot->set_robot_pos_ref(current_ref);

}

bool XBot::XBotCubicTrajectoryGenerator::close(void)
{
    DPRINTF("XBotCubicTrajectoryGenerator close()\n");
}
