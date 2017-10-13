/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:  Giuseppe Rigano
 * email:   giuseppe.rigano@iit.it
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

#ifndef __WEB_ROBOT_STATE_WEBSERVER_H__
#define __WEB_ROBOT_STATE_WEBSERVER_H__

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <vector>

using namespace rapidjson;

class WebRobotState {
  
  public:
    
    std::vector<std::string> joint_name;
    std::vector<int> joint_id;
    std::vector<double> link_position;
    std::vector<double> motor_position;
    std::vector<double> link_vel;
    std::vector<double> motor_vel;
    std::vector<double> temperature;
    std::vector<double> effort;
    
    //XBot::RobotState::pdo_rx pdo_rx;
    //IMU
    //FT
  
    void serialize(StringBuffer& buffer);
    
  private:
    
    //template <typename T>
    void serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<double>& array);
    
    void serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<int>& array);
    
    void serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<std::string>& array);
  
};

#endif //__WEB_ROBOT_STATE_WEBSERVER_H__