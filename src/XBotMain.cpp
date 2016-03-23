#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <XBotCore/XBotPluginHandler.h>

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

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2) {
	printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    threads["boards_ctrl"] = new XBot::XBotPluginHandler(argv[1]);
    threads["boards_ctrl"]->create(true);

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


