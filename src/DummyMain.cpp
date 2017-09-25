#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <ctime>
#include <exception>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <XCM/XBotPluginHandler.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


int main(int argc, char **argv){

    ros::init(argc, argv, "DummyMain");
    ros::NodeHandle nh;

    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);


    using namespace XBot;

    std::string path_to_cfg;

    if(argc > 1){
        path_to_cfg = std::string(argv[1]);
    }
    else{

    }

    std::string framework = "DUMMY";

    if(argc > 2){
        framework = std::string(argv[2]);
    }
    else{

    }


    RobotInterface::Ptr robot = RobotInterface::getRobot(path_to_cfg, AnyMapPtr(), framework);


    auto time_provider = std::make_shared<SimpleTimeProvider>();
    PluginHandler plugin_handler(robot, time_provider, "XBotRTPlugins");

    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();

    plugin_handler.load_plugins();
    plugin_handler.init_plugins(shared_memory);

    double time = 0;
    
    ros::Rate loop_rate(1000);
    
    namespace boost_acc = boost::accumulators;
    
    boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::rolling_mean> > time_acc(boost_acc::tag::rolling_window::window_size = 1000);


    while(ros::ok()){

        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(time);
        clock_pub.publish(msg);

        time_provider->set_time(time);

        if( int(time*1000) % 1000 == 0 ){
            std::cout << "Time [s]: " << int(time) << std::endl;
            std::cout << "Mean exec time [ms]: " << 1000*boost_acc::rolling_mean(time_acc) << std::endl;
        }

        plugin_handler.run();
        
        time += loop_rate.expectedCycleTime().toSec();
        
        time_acc(loop_rate.cycleTime().toSec());

        loop_rate.sleep();

    }

    plugin_handler.close();

}