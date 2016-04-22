#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <XBotCore/XBotCommunicationHandler.h>

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

    if ( argc != 2) {
	printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    XBot::XBotCommunicationHandler* YarpCH = new XBot::XBotCommunicationHandler();
    YarpCH->init(argv[1]);

    while (main_loop) {
        sleep(1);
    }

    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


