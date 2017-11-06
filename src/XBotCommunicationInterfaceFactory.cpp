/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
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

#include <XCM/XBotCommunicationInterfaceFactory.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

std::map<std::string, void*> CommunicationInterfaceFactory::handles;


std::shared_ptr<XBot::CommunicationInterface> CommunicationInterfaceFactory::getFactory(const std::string& file_name,
                                                                                        const std::string& lib_name,
                                                                                        XBot::RobotInterface::Ptr _robot )
{

    char *error;  
    std::string path_to_so;  
    computeAbsolutePath(file_name, "/build/install/lib/", path_to_so);
    path_to_so += std::string(".so");
    void* lib_handle;
    lib_handle = dlopen(path_to_so.c_str(), RTLD_NOW);
    if (!lib_handle) {
        Logger::error() << lib_name << " INTERFACE NOT found! \n" << dlerror() << Logger::endl();
    }
    else     
    {
        Logger::success(Logger::Severity::MID) << lib_name <<" INTERFACE found! " << Logger::endl();
        handles[file_name] = lib_handle;
      
        XBot::CommunicationInterface* (*create)(XBot::RobotInterface::Ptr);
        create = (XBot::CommunicationInterface* (*)(XBot::RobotInterface::Ptr))dlsym(lib_handle, "create_instance");
        if ((error = dlerror()) != NULL) {
            fprintf(stderr, "%s\n", error);
            exit(1);
        }
        
        XBot::CommunicationInterface* instance =(XBot::CommunicationInterface*)create(_robot);
        if( instance != nullptr){
          return std::shared_ptr<XBot::CommunicationInterface>(instance);
        }
     }
    return nullptr;
    
}

void CommunicationInterfaceFactory::unloadLib(const std::string& file_name)
{

  dlclose( handles[file_name] );
  Logger::info() << file_name <<" INTERFACE unloaded! " << Logger::endl();
}

bool CommunicationInterfaceFactory::computeAbsolutePath (  const std::string& input_path,
                                                        const std::string& middle_path,
                                                        std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}
