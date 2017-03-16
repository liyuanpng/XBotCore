/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
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

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <exception>

#include <XCM/XBotCommunicationHandler.h>

volatile sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
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

    threads["ch"] = new XBot::CommunicationHandler(argv[1]);
    threads["ch"]->create(false, 3);
     
    /* Register SIGINT handler AFTER FRAMEWORK HAVE BEEN INITILIZED */
    signal(SIGINT, sigint_handler);
    int num_iters = 0;
    
    while (g_loop_ok) {
        num_iters++;
        sleep(1); 
    }
    
    std::cout << "main_loop " <<  g_loop_ok << num_iters << std::endl;
    
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


