#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <ctime>
#include <exception>
#include <vector>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <XCM/XBotPluginHandler.h>
#include <XBotInterface/Utils.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int main(int argc, char **argv){

    ros::init(argc, argv, "DummyMain");
    ros::NodeHandle nh;

    nh.setParam("/use_sim_time", true);
    
    /* Command line parsing */
    
    std::string path_to_cfg;
    bool provide_clock = false;
    
    {
        po::options_description desc("Locomotion server. Available options:");
        desc.add_options()
            ("config,C", po::value<std::string>(),"config file with custom parameters")
            ("clock,P", "provide /clock messages and set /use_sim_time = true")
        ;

        
        po::positional_options_description p;
        p.add("config", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(desc).positional(p).run(), vm);
        po::notify(vm);

        if (vm.count("config")) {
            path_to_cfg = fs::absolute(vm["config"].as<std::string>()).string();
        }
        else{
            std::cout << desc << std::endl;
            return -1;
        }
        
        if (vm.count("clock")) {
            provide_clock = true;
        }
    }
    
    
 // config file handling
 
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

    if(provide_clock){
        nh.setParam("/use_sim_time", true);
    }
    
    
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);


    using namespace XBot;


    std::string framework = "DUMMY";
	std::cout << "Using config file : " << path_to_cfg << std::endl;
    RobotInterface::Ptr robot = RobotInterface::getRobot(path_to_cfg, "", AnyMapPtr(), framework);

    XBot::SharedMemory::Ptr shared_memory = std::make_shared<XBot::SharedMemory>();

    auto time_provider = std::make_shared<SimpleTimeProvider>();
    XBot::Options options;
    
    PluginHandler plugin_handler(robot, time_provider, shared_memory, options);

    

    plugin_handler.load_plugins();
    plugin_handler.init_plugins();

    double time = 0;
    
    ros::Rate loop_rate(1000);
    
    namespace boost_acc = boost::accumulators;
    
    boost_acc::accumulator_set<double, boost_acc::stats<boost_acc::tag::rolling_mean> > time_acc(boost_acc::tag::rolling_window::window_size = 1000);


    while(ros::ok()){

        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(time);
        
        if(provide_clock){
            clock_pub.publish(msg);
        }
        
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
    
    if(provide_clock){
        nh.setParam("/use_sim_time", false);
    }

    plugin_handler.close();

}
