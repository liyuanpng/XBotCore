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
#include <XBotInterface/RtLog.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

extern void main_common(__sighandler_t sig_handler);

static int main_loop = 1;


void shutdown(int sig __attribute__((unused)))
{
    main_loop = 0;
}

bool getDefaultConfig(std::string& config) 
{
    config = XBot::Utils::getXBotConfig();
    if(config == "") {
        return false;
    }
    return true;
}

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {
    
    using XBot::Logger;
    
    /* Command line parsing */
    
    std::string path_to_cfg;
    std::string path_to_ch_cfg;
    
    XBot::Options options;
    
    {
        po::options_description desc("XBotCore. Available options");
        desc.add_options()
            ("config,C", po::value<std::string>(),"Path to custom config file. If not set, a default config file must be configured via set_xbot_config.")
            ("ch-config", po::value<std::string>(),"Path to custom config file for the CommunicationHandler. If not set, the same of XBotCore is used.")
            ("no-ch", "Do not run the CommunicationHandler")
            ("dont-catch", "Disable catching exceptions thrown inside rt plugins init()")
            ("period", po::value<int>(), "XBotCore thread period [us]")
            ("ch-period", po::value<int>(), "CommunicationHandler thread period [us]")
            ("dummy,D", "Use the dummy HAL implementation.")
            ("verbose,V", "Verbose mode.")
            ("help", "Shows this help message.")
            
        ;

        
        po::positional_options_description p;
        p.add("config", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(desc).positional(p).run(), vm);
        po::notify(vm);
        
        if(vm.count("help")){
            std::cout << desc << std::endl;
            return 0;
        }
        
        if(vm.count("verbose")){
            Logger::SetVerbosityLevel(Logger::Severity::LOW);
            Logger::info("Verbose mode ON \n");
        }
        else{
            Logger::SetVerbosityLevel(Logger::Severity::MID);
        }

        if(vm.count("no-ch")) {
            options.comm_handler_thread_enabled = false;
        }
        
        if(vm.count("config")) {
            path_to_cfg = fs::absolute(vm["config"].as<std::string>()).string();
        }
        else{
            if(!getDefaultConfig(path_to_cfg)){
                std::cout << desc << std::endl;
                return -1;
            }
        }
        
        if(vm.count("ch-config")){
            path_to_ch_cfg = vm.at("ch-config").as<std::string>();
        }
        else{
            path_to_ch_cfg = path_to_cfg;
        }
        
        if(vm.count("dummy")) {
            options.xbotcore_dummy_mode = true;
        }
        
        if(vm.count("dont-catch")){
            options.xbotcore_pluginhandler_catch_exceptions = false;
        }
        
        if(vm.count("period")){
            options.xbotcore_period_us = vm.at("period").as<int>();
        }
        
        if(vm.count("ch-period")){
            options.comm_handler_period_us = vm.at("ch-period").as<int>();
        }
        
    }


    Logger::info(Logger::Severity::HIGH) << XBot::bold_on << "XBotCore using config file " << path_to_cfg << Logger::endl();

    main_common(shutdown);

    auto shared_memory = std::make_shared<XBot::SharedMemory>();

    XBot::CommunicationHandler ch( path_to_ch_cfg.c_str(), shared_memory, options );
    XBot::XBotCoreThread xbc( path_to_cfg.c_str(), shared_memory, options );
    
    
    xbc.create(true, 2);
    
    if(options.comm_handler_thread_enabled){
        ch.create(false, 3);
    }
  
    while (main_loop) {
        sleep(1); 
    }
    
    Logger::info(Logger::Severity::HIGH) << "XBotCore exiting after " <<  main_loop << " seconds. Joining threads..." << Logger::endl();
    

    xbc.stop();
    xbc.join();
    
    if(options.comm_handler_thread_enabled){
        ch.stop();
        ch.join();
    }
    
    XBot::MatLogger::FlushAll();

    Logger::info() << "XBotMain exiting" << Logger::endl();

    return 0;

} catch (std::exception& e) {

    XBot::Logger::error() << "Main catch exception: " <<  e.what() << XBot::Logger::endl();

}


