#include <XCM/XBotPluginHandler.h>
#include <csignal>

sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
}

int main(int argc, char **argv){

    using namespace XBot;

    signal(SIGINT, sigint_handler);

    std::string path_to_cfg(argv[1]);

    RobotInterface::Ptr robot = RobotInterface::getRobot(path_to_cfg, AnyMapPtr(), "DUMMY");

    PluginHandler plugin_handler(robot, path_to_cfg);

    plugin_handler.load_plugins();
    plugin_handler.init_plugins();

    double time = 0;

    while(g_loop_ok){

        plugin_handler.run(time);
        time += 0.001;
        
    }

    plugin_handler.close();

}