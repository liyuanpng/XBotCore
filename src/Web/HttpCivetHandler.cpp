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

#include <HttpCivetHandler.h>
#include <memory>


bool HttpCivetHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
      
      const char* uri =mg_get_request_info(conn)->request_uri;
      std::string suri(uri);
      http_interface->setUri(suri);
      
      const char* query = mg_get_request_info(conn)->query_string;
      if (query!=nullptr){
          std::string squery(query);
          http_interface->setQuery(squery);
          std::string key = squery.substr(0, squery.find("="));
          std::string val = squery.substr(squery.find("=")+1);
          http_interface->setKey(key);
          http_interface->setVal(val);
      }
          
      std::shared_ptr<ResponseInterface> resp_interface;
      http_interface->handleGet(resp_interface);
      int content_length = resp_interface->GetLength();
      //std::cout<<"SIZE "<< f<< std::endl;
      std::string type = resp_interface->GetTypeResponse();
      std::string header = "HTTP/1.1 200 OK\r\nContent-Type: "
                            +type+"; charset=utf-8"+"\r\nContent-Length: "+ 
			    std::to_string(content_length) +"\r\nConnection: close\r\n\r\n";
			    
     
     

      mg_printf(conn, header.c_str());
      mg_write(conn,resp_interface->GetData(),resp_interface->GetLength());
  
      return true;
}  

bool HttpCivetHandler::handlePost(CivetServer* server, mg_connection* conn){
  
      const char* uri =mg_get_request_info(conn)->request_uri;
      std::string suri(uri);
      http_interface->setUri(suri);
      
      const struct mg_request_info *req_info = mg_get_request_info(conn);
      long long rlen = 0;
      long long nlen = 0;
      long long tlen = req_info->content_length;
      //std::cout<<"tlen "<<tlen<<std::endl;
      //NOTE fixed size
      char buf[4096];
  
      memset(buf,0,4096);
      while (nlen < tlen) {
          rlen = tlen - nlen;
          if (rlen > sizeof(buf)) {
                  rlen = sizeof(buf);
          }
          rlen = mg_read(conn, buf, (size_t)rlen);
          if (rlen <= 0) {
                  break;
          }
         
       }
      
      std::shared_ptr<RequestObject> req = std::make_shared<RequestObject>();
      std::string type = "application/json"; //TODO make general
      std::string header = "HTTP/1.1 200 OK\r\nContent-Type: "
                            +type+"\r\nConnection: close\r\n\r\n";
      req->SetData(buf);
      http_interface->handlePost(req);
      mg_printf(conn, header.c_str());
      mg_printf(conn,"{\"response\":\"ok\"}");
  
      return true;
}  
  