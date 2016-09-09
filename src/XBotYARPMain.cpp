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

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <XBotCore/XBotCommunicationHandler.h>
#include <XBotCore/IXBotCommunication.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <map>
#include <vector>

#define WRAPPER_RATE 5
#define ANALOG_SERVER_RATE 5

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;

void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");
}

void YARP_configuration(const std::shared_ptr<XBot::XBotCommunicationHandler>& ch,
                        std::map<std::string, yarp::dev::PolyDriver >& motion_control_map,
                        std::map<std::string, yarp::dev::PolyDriver >& wrapper_map,
                        std::map<std::string, yarp::dev::PolyDriver >& ft_map,
                        std::map<std::string, yarp::dev::PolyDriver >& analog_server_map) 
{

    // get the robot chain/joints
    std::map<std::string, std::vector<int> > robot = ch->get_robot_map();
    // get the robot FT
    std::map<std::string, int> ft = ch->get_ft_sensors_map();
    // define XBotCommunication interface
    XBot::IXBotCommunication* xbot_communication;
    
    // iterate over the actual robot in order to define dynamic wrappers
    for(auto& c : robot) {
        
        // joints in chain
        yarp::os::Bottle joints;   
        yarp::os::Bottle joint_groups;  
        for(int i = 0; i < c.second.size(); i++) {
            joints.addInt(c.second[i]);
        }
        
        // open motion control device with chain joint map
        yarp::os::Property mc_config;
        mc_config.put("device","XBotMotionControl");    
        mc_config.put("joint_map", joints.toString()); 
        mc_config.put("chain_name", c.first); 
        motion_control_map[c.first].open(mc_config);
        
        // init
        motion_control_map.at(c.first).view(xbot_communication);
        xbot_communication->init(ch);
        
        // defining control board wrapper
        yarp::os::Property wr_config;    
        wr_config.put("device","controlboardwrapper2"); 
        wr_config.put("robot_name", "bigman"); 
        wr_config.put("name", "/" + wr_config.find("robot_name").asString() + "/" + c.first);
        wr_config.put("period", WRAPPER_RATE);           
        
        // NOTE are you crazy, Mr YARP?
        yarp::os::Bottle chains; 
        yarp::os::Bottle& actual_chain_list = chains.addList();     
        actual_chain_list.addString("networks");
        yarp::os::Bottle& actual_chain = actual_chain_list.addList();
        actual_chain.addString(c.first);
        // wrapper chains config
        wr_config.fromString(chains.toString(), false);
        
        // wrapper joints config
        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        joint_groups.addInt(0);
        joint_groups.addInt(joints.size() - 1);
        wr_config.put(c.first, joint_groups.toString());
        wr_config.put("joints", joints.size()); 
        DPRINTF("wr_config joints : %s\n", wr_config.toString().c_str());
        wrapper_map[c.first].open(wr_config);
        
        // view on the wrapper and attach the poly driver
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = c.first;
        poly_descriptor.poly = &motion_control_map.at(c.first);
        poly_list.push(poly_descriptor);
        
        wrapper_map.at(c.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
    }
    
    // iterate over the actual robot FT in order to define dynamic wrappers
    for(auto& ft_j : ft) {
        
        // open motion control device with chain joint map
        yarp::os::Property ft_config;
        ft_config.put("device","XBotFT");   
        ft_config.put("channels", 6); //TBD take it from a config file 
        ft_config.put("ft_name", ft_j.first); 
        ft_map[ft_j.first].open(ft_config);
        
        // init
        ft_map.at(ft_j.first).view(xbot_communication);
        xbot_communication->init(ch);
        
        // defining control board wrapper
        yarp::os::Property wr_config;    
        // NOTE hack
        std::string aux = ft_j.first;
        aux = aux.substr(0, aux.size() - 6);
        
        wr_config.put("device","analogServer"); 
        wr_config.put("robot_name", "bigman"); // TBD GET FROM SOMEWHERE 
        wr_config.put("name", "/" + wr_config.find("robot_name").asString() + "/" + aux + "/" + "analog:o/forceTorque");
        wr_config.put("period", ANALOG_SERVER_RATE);            // TBD do it from config YAML
        analog_server_map[ft_j.first].open(wr_config);
        
        // view on the wrapper and attach the poly driver
        yarp::dev::PolyDriverDescriptor poly_descriptor;
        yarp::dev::PolyDriverList poly_list;
        yarp::dev::IMultipleWrapper* multi_wrapper;
        
        poly_descriptor.key = ft_j.first;
        poly_descriptor.poly = &ft_map.at(ft_j.first);
        poly_list.push(poly_descriptor);
        
        analog_server_map.at(ft_j.first).view(multi_wrapper);
        multi_wrapper->attachAll(poly_list);
        
    }
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {
    

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2) {
	DPRINTF("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    std::shared_ptr<XBot::XBotCommunicationHandler> commHandler = std::make_shared<XBot::XBotCommunicationHandler>(argv[1]);
    threads["boards_ctrl"] = commHandler.get();
    threads["boards_ctrl"]->create(false); // N-RT thread
    
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        DPRINTF("yarpserver not running - run yarpserver\n");
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();
    

            
    // create YARP MotionControl and Wrappers (coupled)
    std::map<std::string, yarp::dev::PolyDriver > motion_control_map;
    std::map<std::string, yarp::dev::PolyDriver > wrapper_map;
    // ft
    std::map<std::string, yarp::dev::PolyDriver > ft_map;
    std::map<std::string, yarp::dev::PolyDriver > analog_server_map;
    
    // YARP configuration
    YARP_configuration(commHandler, 
                       motion_control_map, 
                       wrapper_map,
                       ft_map,
                       analog_server_map);


    while (main_loop) {
        sleep(1);
    }

    for ( auto const& item : threads ) {
        item.second->stop();
        item.second->join();
        delete item.second;
    }
    
    std::cout << "Exit main" << std::endl;
    
    yarp.fini();

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


