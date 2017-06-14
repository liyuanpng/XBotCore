#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <ctime>
#include <exception>

#include <XCM/XBotPluginHandler.h>

static sigset_t   signal_mask;
volatile sig_atomic_t g_loop_ok = 1;

void *signal_thread (void *arg)
{
    int       sig_caught;    /* signal caught       */
    int       rc;            /* returned code       */


    rc = sigwait (&signal_mask, &sig_caught);
    if (rc != 0) {
        /* handle error */
    }
    switch (sig_caught)
    {
    case SIGINT:     /* process SIGINT  */
        g_loop_ok = 0;
        break;
    case SIGTERM:    /* process SIGTERM */
        g_loop_ok = 0;
        break;
    default:         /* should normally not happen */
        fprintf (stderr, "\nUnexpected signal %d\n", sig_caught);
        break;
    }
}

int main(int argc, char **argv){

    // SIGNAL handling
    pthread_t  sig_thr_id;
    int rc;

    sigemptyset (&signal_mask);
    sigaddset (&signal_mask, SIGINT);
    sigaddset (&signal_mask, SIGTERM);
    rc = pthread_sigmask (SIG_BLOCK, &signal_mask, NULL);
    rc = pthread_create (&sig_thr_id, NULL, signal_thread, NULL);
    // SIGNAL handling


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

    while(g_loop_ok){

        std::clock_t tic = std::clock();

        time_provider->set_time(time);

        if( int(time*1000) % 1000 == 0 ){
            std::cout << "Time: " << int(time) << std::endl;
        }

        plugin_handler.run();
        time += 0.001;

        double elapsed_time = double(std::clock() - tic)/CLOCKS_PER_SEC;

        usleep(std::max(.0, 1000 - elapsed_time*1e6));

    }

    plugin_handler.close();

}