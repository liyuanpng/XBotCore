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

#include <XCM/Loader.h>
#include <boost/bind.hpp>
#include <string>
#include <atomic>

std::atomic<bool> t;

std::string Loader::name;

bool Loader::callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res, const std::string& port_name)
{    
    res.success = true;
    std::cout<<"restart "<<port_name<<std::endl;

    Loader::name = port_name;
    t.store(true);    
    return true;
}

void Loader::init_internal()
{

    int argc = 1;
    const char *arg = "Loader";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "Loader_" + std::to_string(XBot::get_time_ns()),ros::init_options::NoSigintHandler);
    }
      
    nh = std::make_shared<ros::NodeHandle>();
    
    for( const std::string port_name : _pluginHandler->getPluginsName()){      
    
      _services[ port_name] = nh->advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(port_name+"_restart",  boost::bind(&Loader::callback,this,_1,_2,port_name));
      
    }
    
    t.store(false);
}

void Loader::loop_internal()
{

   if(t.load())
      _pluginHandler->replacePlugin(Loader::name);
    t.store(false);

    ros::spinOnce();
    fflush(stdout);
  
}

Loader::Loader(XBot::PluginHandler::Ptr pluginH): 
                _pluginHandler(pluginH)
                
{
  
    
}

Loader::~Loader()
{

}
