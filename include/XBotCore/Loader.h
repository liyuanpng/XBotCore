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

#ifndef __LOADER_H__
#define __LOADER_H__

#include <XCM/XBotPluginHandler.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class Loader {
    
public:
  Loader(){};
  Loader(XBot::PluginHandler::Ptr,
         XBot::SharedMemory::Ptr shared_memory,
        std::shared_ptr< XBot::IXBotJoint> joint    = nullptr,
        std::shared_ptr< XBot::IXBotFT > ft         = nullptr,
        std::shared_ptr< XBot::IXBotIMU > imu       = nullptr,
        std::shared_ptr< XBot::IXBotHand > hand     = nullptr ,
        std::shared_ptr< XBot::IXBotModel > model   = nullptr 
  );
  
  ~Loader();
  
  void operator()();

  void doit(const std::string& port_name);
    static std::string name;
private:
    
  XBot::PluginHandler::Ptr _pluginHandler;
  XBot::SharedMemory::Ptr _shared_memory;
  std::shared_ptr< XBot::IXBotJoint> _joint;
  std::shared_ptr< XBot::IXBotFT > _ft;
  std::shared_ptr< XBot::IXBotIMU > _imu;
  std::shared_ptr< XBot::IXBotHand > _hand;
  std::shared_ptr< XBot::IXBotModel > _model;
  
  bool callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res, const std::string& port_name);
  
  std::shared_ptr<ros::NodeHandle> nh;
  std::map<std::string, ros::ServiceServer> _services;
  
  
};
#endif