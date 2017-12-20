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

#ifndef __RESPONSE_WEBSERVER_H__
#define __RESPONSE_WEBSERVER_H__

#include <ResponseInterface.h>
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

class StringResponse : public ResponseInterface {
  
  private:
    
      std::string response;
    
  public:
    
    StringResponse(std::string resp){
      
      response = resp;
    }
    
    const void * GetData() {
      
      return response.c_str();
      
    };
    
    std::size_t GetLength() {
      
      return response.length();
    };
    
    std::string GetTypeResponse(){
      
      std::string s = ("text/html");
      return s;
      
    }
  
};

class JsonResponse : public ResponseInterface {
  
  private:
  
      std::shared_ptr<StringBuffer> response;
      
  public:
    
      JsonResponse(std::shared_ptr<StringBuffer>& buffer){
        
        response = buffer;
      } 
      
      const void * GetData() {
        
        return (const void*)response->GetString();
      };
      
      std::size_t GetLength() {
        
        return response->GetLength();
      };
      
      std::string GetTypeResponse(){
        
        std::string s ="application/json";
        return s;
      
    }
    
};

#endif //__RESPONSE_WEBSERVER_H__