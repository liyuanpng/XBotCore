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
         
      std::shared_ptr<StringBuffer> jsonresp = std::make_shared<StringBuffer>();
      jsonresp->Clear();
      Writer<StringBuffer> writer(*jsonresp);
  
      if(uri.compare("/switch")==0){
        sharedData->insertSwitch(key, val);
	writer.StartObject();  
	writer.Key("Response");
	//TODO check return value insertswitch and answer accordingly
	writer.String("OK");
	writer.EndObject();
      }
      else if(uri.compare("/status")==0){
	writer.StartObject();  
	writer.Key("Response");
	std::string s =sharedData->getAllStatus()[key];
	if (s.empty()) s = "Error: Not Found";
	writer.String(s.c_str());  
	writer.EndObject();
      }
      else if(uri.compare("/cmd")==0) {
        sharedData->insertCmd(key, val);
	writer.StartObject();  
	writer.Key("Response");
	writer.String("OK");
	writer.EndObject();
      }       
      else if(uri.compare("/master")==0){
        sharedData->setMaster(key);
	writer.StartObject();  
	writer.Key("Response");
	writer.String("OK");
	writer.EndObject();
      }
      else if(uri.compare("/plugins")==0){
	        
	writer.StartObject();  
	writer.Key("Plugins");   
	writer.StartArray();
	for( auto const &s : sharedData->getAllStatus()){                 
	    const std::string& outer_key = s.first;
	    const std::string& inner_map = s.second;
	    writer.StartObject();  
	    writer.Key("Name");
	    writer.String(outer_key.c_str());
	    writer.Key("Status");
	    writer.String(inner_map.c_str());
	    writer.EndObject();
	}
	writer.EndArray();
	writer.EndObject();  	
      }
      else if(uri.compare("/state")==0){
	      
	WebRobotState rstate;
	bool resp = buffer->remove(rstate);
	if(resp){      
	    rstate.serialize(*jsonresp);
	}
      }
      else if(uri.compare("/chains")==0){
	      
	writer.StartObject();  
	writer.Key("Chains");   
	writer.StartArray();
	for( auto const &pair : sharedData->getChainMap()){                 
	    const std::string& outer_key = pair.first;
	    const std::vector<std::string>& inner_map = pair.second;
	    writer.StartObject();  
	    writer.Key("Chain"); 
	    writer.String(outer_key.c_str());
	     writer.Key("Val"); 
	    writer.StartArray();
	    for( std::string s: inner_map){
	      std::size_t found = s.find("|");
	      writer.StartObject(); 
	      writer.Key("Name");
	      writer.String(s.substr(0,found).c_str());
	      writer.Key("ID");
	      writer.Int(std::stoi(s.substr(found+1)));
	      writer.EndObject();
	    }
	    writer.EndArray();;
	    writer.EndObject();
	}
	writer.EndArray();
	writer.EndObject();  	
      }
      response = std::make_shared<JsonResponse>(jsonresp);
}
  
void HttpHandler::handlePost(std::shared_ptr<RequestObject>& binary_request){
    
      std::unique_ptr<JsonRequest> getter = std::unique_ptr<JsonRequest>(new JsonRequest(binary_request));
//       void* buff = binary_request->GetData();     
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
  