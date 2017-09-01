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

#include <HttpHandler.h>

HttpHandler::HttpHandler (std::shared_ptr<SharedData>& sharedData, std::shared_ptr<Buffer<WebRobotState>>& buffer){
      
      this->sharedData = sharedData;
      this->buffer = buffer;
}
 
void HttpHandler::handleGet(std::shared_ptr<ResponseInterface>& response){      
         
      if(uri.compare("/switch")==0){
        sharedData->insertSwitch(key, val);       
      }
      else if(uri.compare("/cmd")==0) {
        sharedData->insertCmd(key, val);        
      }       
      else if(uri.compare("/webmaster")==0){
        sharedData->setMaster(key);
      }
      
      std::string sresp="";
      sresp+="<html><body>\r\n";
      sresp+=("<h2>XBOTCORE </h2>\r\n");
      
      for( auto const &s : sharedData->getAllStatus()){                 
          auto const &outer_key = s.first;
          auto const &inner_map = s.second;
          std::string ss="<h3>"+ outer_key+ " "+ inner_map+ "</h3>\r\n";
          const char * w =const_cast<char*>( ss.c_str());
          sresp+=(w);
      }

      sresp+=("</body></html>\r\n");

      response = std::make_shared<StringResponse>(sresp);
     
//       std::shared_ptr<StringBuffer> jsonresp = std::make_shared<StringBuffer>();
//       // 1. Parse a JSON string into DOM.
//       const char* json = "[{\"project\":\"rapidjson\",\"stars\":10}, {\"project\":\"rapidjson\",\"stars\":10}]";
//       Document d;
//       d.Parse(json);
// 
//       // 2. Modify it by DOM.
//       Value& s = d["stars"];
//       s.SetInt(s.GetInt() + 1);
// 
//       // 3. Stringify the DOM
//       Writer<StringBuffer> writer(*jsonresp);
//       d.Accept(writer);
//       
//       response = std::make_shared<JsonResponse>(jsonresp);
     
             
}
  
void HttpHandler::handlePost(std::shared_ptr<RequestObject>& binary_request){
    
      std::unique_ptr<JsonRequest> getter = std::unique_ptr<JsonRequest>(new JsonRequest(binary_request));
//       void* buff = request->GetData();     
//       std::cout<<"pos"<<std::string((char*)buff)<<std::endl;

      sharedData->clearJointMap();
      
      std::vector<double> vec;
      std::map<int, double> map;
      if(uri.compare("/alljoints")==0){     
        
        if(getter->GetDoubleArray("link_position", vec)){       
          sharedData->external_command->add(vec);
          //HACK simulation of holding value for longer time
          sharedData->external_command->add(vec);
        }
        
      }else if(uri.compare("/singlejoint")==0){
        
        //{"joint":[{"id": 15, "val": 0},{"id": 16, "val": 0}]}
        if(getter->GetIntDoubleMap("joint", map)){
          for( auto& ref : map){
            sharedData->insertJoint(ref.first,ref.second);
          }
        }        
      }
    
}
  