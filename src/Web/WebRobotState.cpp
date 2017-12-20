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

#include <WebRobotState.h>

void WebRobotStateTX::serialize(StringBuffer& buffer){
      
        Writer<StringBuffer> writer(buffer);
        writer.StartObject();  
        serializeArray(writer,"joint_name",joint_name);
        serializeArray(writer,"joint_id",joint_id);
        serializeArray(writer,"link_position",link_position);
        serializeArray(writer,"motor_position", motor_position);
        serializeArray(writer,"link_velocity",link_vel);
        serializeArray(writer,"motor_velocity",motor_vel);
        serializeArray(writer,"temperature",temperature);   
	serializeArray(writer,"effort",effort); 
        serializeArray(writer,"stiffness",stiffness);
        serializeArray(writer,"damping",damping);
        writer.EndObject();  
}

//template <typename T>
void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<double>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( double val : array ){  
      writer.Double(val);
    }
    writer.EndArray();
    // writer.EndObject();  
}

void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<int>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( int val : array ){  
      writer.Int(val);
    }
    writer.EndArray();
    // writer.EndObject();  
}

void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<std::string>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( std::string& val : array ){  
      writer.String(val.c_str());
    }
    writer.EndArray();
    // writer.EndObject();  
}