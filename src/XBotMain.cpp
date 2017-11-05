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

#include <XBotCore/XBotCoreThread.h>
#include <XBotCore/XBotLoaderThread.h>

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

    std::map<std::string, XBot::Thread_hook*> threads;
    if ( argc < 2) {
        printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common(shutdown);
    
    threads["boards_ctrl"] = new XBot::XBotCoreThread(argv[1], argv[2]);
    threads["boards_ctrl"]->create(true, 2);
  
//     threads["loader"] = new XBot::XBotLoaderThread();
//     threads["loader"]->create(false, 2);

    while (main_loop) {
        sleep(1); 
    }
    
    std::cout << "main_loop " <<  main_loop << std::endl;
    
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


