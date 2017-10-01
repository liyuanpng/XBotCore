#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <ctime>
#include <exception>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <XCM/XBotPluginHandler.h>
#include <XBotInterface/Utils.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


int main(int argc, char **argv){

    ros::init(argc, argv, "DummyMain");
    ros::NodeHandle nh;

    nh.setParam("/use_sim_time", true);
    
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);


    using namespace XBot;

    std::string framework = "DUMMY";

    // config file handling
    std::string path_to_cfg;
    if ( argc != 2 ) {
        // check the default path
        path_to_cfg = XBot::Utils::getXBotConfig();
        if(path_to_cfg == "") {
            printf ( "Usage: %s config.yaml\nOr set_xbot_config config.yaml && %s\n", argv[0], argv[0] );
            return 0;
        }
    }
    else {
        path_to_cfg = argv[1];
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
        
        std::clock_t tic = std::clock();

        time_provider->set_time(time);

        if( int(time*1000) % 1000 == 0 ){
            std::cout << "Time [s]: " << int(time) << std::endl;
            std::cout << "Mean exec time [ms]: " << 1000*boost_acc::rolling_mean(time_acc) << std::endl;
        }
        

        plugin_handler.run();
        
        time += 0.001;
        
        double elapsed_time = double(std::clock() - tic)/CLOCKS_PER_SEC;
        
        time_acc(elapsed_time);

        usleep(std::max(.0, 1000 - elapsed_time*1e6));

        
        


    }

    plugin_handler.close();

}