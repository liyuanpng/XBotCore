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
#include <XBotInterface/Utils.h>

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

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main ( int argc, char *argv[] ) try {

    // SIGNAL handling
    pthread_t  sig_thr_id;
    int rc;

    sigemptyset (&signal_mask);
    sigaddset (&signal_mask, SIGINT);
    sigaddset (&signal_mask, SIGTERM);
    rc = pthread_sigmask (SIG_BLOCK, &signal_mask, NULL);
    rc = pthread_create (&sig_thr_id, NULL, signal_thread, NULL);
    // SIGNAL handling

    
    int num_iters = 0;
    std::map<std::string, XBot::Thread_hook*> threads;
    
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

    auto shmem = std::make_shared<XBot::SharedMemory>();
    XBot::Options options;

    threads["ch"] = new XBot::CommunicationHandler ( path_to_cfg, shmem, options);
    threads["ch"]->create ( false, 3 );
    



    while ( g_loop_ok ) {
        num_iters++;
        sleep ( 1 );
    }

    std::cout << "main_loop: exiting after " << num_iters << " seconds. " << std::endl;

    for ( auto const& item : threads ) {
        std::cout << "Trying to stop thread : " << item.first.c_str() << std::endl;
        item.second->stop();
        item.second->join();
        delete item.second;
    }


    std::cout << "Exit main" << std::endl;

    return 0;

} catch ( std::exception& e ) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


