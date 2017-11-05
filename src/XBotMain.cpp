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

#include <XCM/XBotCommunicationHandler.h>

#include <XBotInterface/Utils.h>

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;


void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
    printf("got signal .... Shutdown\n");

}

bool getDefaultConfig(std::string& config, char *argv[]) 
{
    config = XBot::Utils::getXBotConfig();
    if(config == "") {
        printf ( "Usage: %s config.yaml\nOr set_xbot_config config.yaml && %s\n", argv[0], argv[0] );
        return false;
    }
    return true;
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {

    std::map<std::string, XBot::Thread_hook*> threads;
    
    // config file handling
    std::string path_to_cfg;
    char* dummy_arg = argv[2];
    
    if ( argc != 2 ) {
        // check the default path
        if(!getDefaultConfig(path_to_cfg, argv)) {
            return 0;
        }
    }
    else {
            
        // NOTE the user can also call XBotCore dummy -> Config file must have been set through set_xbot_config
        if(strcmp(argv[1], "dummy") == 0){
            dummy_arg = argv[1];
            // check the default path
            if(!getDefaultConfig(path_to_cfg, argv)) {
                return 0;
            }
        }
        else {
            path_to_cfg = argv[1];
        }
    }


    main_common(shutdown);


    XBot::XBotCoreThread xbc( path_to_cfg.c_str(), dummy_arg );
    XBot::CommunicationHandler ch( path_to_cfg.c_str() );
    
    threads["boards_ctrl"] = &xbc;
    threads["boards_ctrl"]->create(true, 2);
    
    threads["ch"] = &ch;
    threads["ch"]->create(false, 3);
  
//     threads["loader"] = new XBot::XBotLoaderThread();
//     threads["loader"]->create(false, 2);

    while (main_loop) {
        sleep(1); 
    }
    
    std::cout << "main_loop " <<  main_loop << std::endl;
    
    for ( auto const& item : threads ) {
        item.second->stop();
        item.second->join();
//         delete item.second;
    }


    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


