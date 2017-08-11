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

#ifndef __RESPONSE_INTERFACE_WEBSERVER_H__
#define __RESPONSE_INTERFACE_WEBSERVER_H__


class ResponseInterface {
  
  public:
    virtual const void * GetData() = 0;
    
    virtual std::size_t GetLength() = 0;
    
    virtual std::string GetTypeResponse() = 0;
    
};

#endif //__RESPONSE_INTERFACE_WEBSERVER_H__