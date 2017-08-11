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

#include <WebSocketCivetHandler.h>
#include <iostream>
#include <assert.h>

bool WebSocketHandler::handleConnection(CivetServer *server, const struct mg_connection *conn) {
  
    std::cout<<"WS connected\n";
    return true;  
}

void WebSocketHandler::handleReadyState(CivetServer *server, struct mg_connection *conn) {
   
    std::cout<<"WS ready\n";
    const char *text = "Hello from XBotCore";
    mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, text, strlen(text));    
    sharedData->increaseNumClient();    
} 
        
bool WebSocketHandler::handleData(CivetServer *server,
                                struct mg_connection *conn,
                                int bits,
                                char *data,
                                size_t data_len) {
  
    data[data_len] = 0;
    std::shared_ptr<RequestObject> req = std::make_shared<RequestObject>();
    req->SetData(data);
    
    //TODO move in abstraction interface
    void * buff;
    buff = req->GetData();     
    StringStream stream((char*)buff);
    //std::cout<<"pos"<<std::string((char*)buff)<<std::endl;
    Document d;
    d.ParseStream(stream);
    
    sharedData->clearJointMap();      
    if( d.HasMember("joint")){          
        assert(d["joint"].isArray());
        const Value& array = d["joint"];
        for (SizeType i = 0; i < array.Size(); i++){
            const Value& obj = array[i];
            int id = obj["id"].GetInt();
            double val = obj["val"].GetDouble();
            sharedData->insertJoint(id,val);              
        }        
      }    
        
    //read robot state
    WebRobotState rstate;
    bool resp = buffer->remove(rstate);
        
    StringBuffer buffer;
    const char* btosend = nullptr;    
    buffer.Clear();
    if(resp){      
        rstate.serialize(buffer);
        btosend = buffer.GetString();
    }
        
    if( btosend!=nullptr)
        mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, btosend, buffer.GetLength());
        
    return true;
}  

void WebSocketHandler::handleClose(CivetServer *server, const struct mg_connection *conn) {
       std::cout<<"WS closed\n";
       sharedData->decreaseNumClient();
}  