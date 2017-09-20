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

#ifndef __HTTP_INTERFACE_WEBSERVER_H__
#define __HTTP_INTERFACE_WEBSERVER_H__

#include <string>
#include <memory>

class ResponseInterface;
class RequestObject;

class HttpInterface {
  
  public:
    
      HttpInterface():uri(""),query(""),key(""),val(""){};
    
      virtual void handleGet(std::shared_ptr<ResponseInterface>& response) = 0;
      
      virtual void handlePost(std::shared_ptr<RequestObject>& request) = 0;
      
      void setUri(std::string& val){
        uri = val;
      }
      
      void setQuery(std::string& val){
        query = val;
      }
      
      void setKey(std::string& val){
        key = val;
      }
      
      void setVal(std::string& val){
        this->val = val;
      }
      
  protected:
      std::string uri;
      std::string query;
      std::string key;
      std::string val;
      //TODO create object httpresponseparse
     
};

#endif //__HTTP_INTERFACE_WEBSERVER_H__