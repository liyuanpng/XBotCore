#include <XCM/XBotPluginHandler.h>
#include <csignal>

sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
}

int main(int argc, char **argv){

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

    signal(SIGINT, sigint_handler);

    
    auto time_provider = std::make_shared<SimpleTimeProvider>();
    PluginHandler plugin_handler(robot, time_provider);

    plugin_handler.load_plugins();
    plugin_handler.init_plugins();

    double time = 0;

    while(g_loop_ok){
        
        time_provider->set_time(time);

        if( int(time*1000) % 1000 == 0 ){
            std::cout << "Time: " << int(time) << std::endl;
        }

        plugin_handler.run();
        time += 0.001;

        usleep(1000);

    }

    plugin_handler.close();

}