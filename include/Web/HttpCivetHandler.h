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

#ifndef __HTTP_CIVETHANDLER_WEBSERVER_H__
#define __HTTP_CIVETHANDLER_WEBSERVER_H__

#include "CivetServer.h"
#include <HttpHandler.h>

class HttpCivetHandler : public CivetHandler
{
  
  private:
    
    std::shared_ptr<HttpInterface> http_interface;
  
  public:
    
    HttpCivetHandler(std::shared_ptr<HttpInterface> interface){
      
      http_interface = interface;
     
    }  
    
    bool handleGet(CivetServer *server, struct mg_connection *conn);
    bool handlePost(CivetServer* server, mg_connection* conn);
};


#endif //__HTTP_CIVETHANDLER_WEBSERVER_H__