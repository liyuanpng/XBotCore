#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <XBotCore/XBotCommunicationHandler.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <map>
#include <vector>

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;

void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {
    
    // TBD check YARP network
    
    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2) {
	printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    XBot::XBotCommunicationHandler* commHandler = new XBot::XBotCommunicationHandler(argv[1]);
    threads["boards_ctrl"] = commHandler;
    threads["boards_ctrl"]->create(false);
    
    // create YARP MotionControl and Wrappers (coupled)
    std::map<std::string, std::vector<int> > robot = commHandler->get_robot_map();
    
    std::map<std::string, yarp::dev::PolyDriver > motion_control_map;
    std::map<std::string, yarp::dev::PolyDriver > wrapper_map;
    
    for(auto& c : robot) {
        // wrapper and motion control configuration
        yarp::os::Bottle joints;   
         
        yarp::os::Bottle joint_groups;  
        for(int i = 0; i < c.second.size(); i++) {
            joints.addInt(c.second[i]);
        }
        yarp::os::Property mc_config;
        mc_config.put("device","XBotMotionControl");
        mc_config.put("joint_map", joints.toString());  
         

        motion_control_map[c.first].open(mc_config);
        
        yarp::os::Property wr_config;    

        wr_config.put("device","controlboardwrapper2");
        wr_config.put("name", "/" + c.first); // TBD put robot name
        wr_config.put("period", 5);     // TBD do it from config YAML
        
        yarp::os::Bottle chains; 
        yarp::os::Bottle& actual_chain1 = chains.addList();     //TBD are you crazy?
        actual_chain1.addString("networks");
        yarp::os::Bottle& actual_chain = actual_chain1.addList();
        actual_chain.addString(c.first);
        DPRINTF("chains %s \n", chains.toString().c_str());
        
        wr_config.fromString(chains.toString(), false);
        DPRINTF("net %s \n", wr_config.toString().c_str());

        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        
        wr_config.put(c.first, joint_groups.toString());
        wr_config.put("joints", joints.size()); 
        DPRINTF("wr_config : %s\n", wr_config.toString().c_str());
        wrapper_map[c.first].open(wr_config);
        
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = c.first;
        poly_descriptor.poly = &motion_control_map.at(c.first);
        poly_list.push(poly_descriptor);
        
        wrapper_map.at(c.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
    }

    

    while (main_loop) {
        sleep(1);
    }

    for ( auto const& item : threads ) {
        item.second->stop();
        item.second->join();
        delete item.second;
    }
    
    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


