/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       Giuseppe Rigano (2017, giuseppe.rigano@iit.it)
       
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
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/

#include <XBotCore/XBotCore.h>
#include <boost/bind.hpp>
// #include <XBotEcat.h>
//#include <Ethernet.h>
#include <Kuka.h>

//TODO example
float JntVals[7];
bool first=false;

XBot::XBotCore::XBotCore(const char* config_yaml) : 
    _path_to_config(config_yaml)
{
    //TODO fix thread stuff
    // set thread name
    //const YAML::Node& board_ctrl = root_cfg["x_bot_ecat"]; // TBD check that the Node is defined
    //set_thread_name(board_ctrl["name"].as<std::string>()); // TBD check that name is defined
    
    set_thread_name("XBOT");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,1};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    
    //TODO use FactoryPattern
    //use shared library
    //halInterface= new XBot::Ethernet(_path_to_config.c_str()); 
//     halInterface = new XBot::XBotEcat(_path_to_config.c_str());
    halInterface = new XBot::Kuka(_path_to_config.c_str());
}

void XBot::XBotCore::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotCore::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotCore::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotCore::set_thread_priority()
{

    // set scheduler policy
#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotCore::th_init( void * ){
  
  halInterface->init();
  //halInterface->get_link_pos(0,val);
  //control_init();
//    for(int i=0;i<LBR_MNJ;i++){
//      double val;
//       halInterface->get_link_pos(i,val);
//       JntVals[i] = val;
//     }
}

void XBot::XBotCore::th_loop( void * ){
  
  halInterface->recv_from_slave();
  control_loop();
  halInterface->send_to_slave();
  
}

void XBot::XBotCore::control_init(void) 
{
    
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(halInterface);
    std::shared_ptr<XBot::IXBotFT> xbot_ft(halInterface);
    std::shared_ptr<XBot::IXBotIMU> xbot_imu(halInterface);
    std::shared_ptr<XBot::IXBotHand> xbot_hand(halInterface);
    
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    
    //TODO use isRT from RobotControlInterface robotInterface.IsRt()
    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::XBotCore::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider, "XBotRTPlugins");  //"XBotRTPlugins"
    
    // define the XBotCore shared_memory for the RT plugins
    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();
    
    //
    _pluginHandler->load_plugins();
    
    //
    _pluginHandler->init_plugins(shared_memory, xbot_joint, xbot_ft, xbot_imu);
}

double XBot::XBotCore::get_time()
{
    return XBot::get_time_ns() / 1e9;
}

int XBot::XBotCore::control_loop(void) 
{    
//     std::cout << "laurenzi" << std::endl;
    _iter++;
   // _pluginHandler->run();  
     //TODO ask if ok or to move 
    if(!halInterface->getState()){ val = 0; return 0;}
    ////////////////
    //EXample
    //read value joints once
    if(!first){
      first=true;
    for(int i=0;i<LBR_MNJ;i++){
     double val;
      halInterface->get_link_pos(i,val);
      JntVals[i] = val;
    }
    }
   
    val+=halInterface->getSampleTime()*0.01;
    for (int i = 0; i < LBR_MNJ; i++)
    {
	// perform some sort of sine wave motion
	JntVals[i]+=(float)sin( val * M_PI * 0.02) * (float)(10./180.*M_PI);
    }  
    
    for(int i=0;i<LBR_MNJ;i++){
      halInterface->set_pos_ref(i,JntVals[i]);
    }
    /////////////

}

XBot::XBotCore::~XBotCore() {
    
    _pluginHandler->close();
    printf("Iteration: %d \n", _iter);
    printf("~XBotCore()\n");
}
