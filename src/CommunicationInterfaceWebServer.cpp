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


#include <XCM/CommunicationInterfaceWebServer.h>
#include <XCM/XBotUtils.h>
#include <iostream>
#include <unistd.h>
#include <cstring>


bool exitNow = false;


namespace XBot {

bool HttpCivetHandler::handleGet(CivetServer *server, struct mg_connection *conn) {
      
      const char* uri =mg_get_request_info(conn)->request_uri;
      std::string suri(uri);
      
      const char* query = mg_get_request_info(conn)->query_string;
      if (query!=nullptr){
	  std::string squery(query);
	  std::string key = squery.substr(0, squery.find("="));
	  std::string val = squery.substr(squery.find("=")+1);
	  http_interface->setKey(key);
	  http_interface->setVal(val);
      }
      std::string squery(query);
        
      http_interface->setUri(suri);
      http_interface->setQuery(squery);
      std::shared_ptr<ResponseInterface> resp_interface;
      http_interface->handleGet(resp_interface);
      std::string type = resp_interface->GetTypeResponse();
      std::string header = "HTTP/1.1 200 OK\r\nContent-Type: "
			    +type+"\r\nConnection: close\r\n\r\n";

      mg_printf(conn, header.c_str());
      mg_write(conn,resp_interface->GetData(),resp_interface->GetLength());
  
      return true;
}  
  
  
bool WebSocketHandler::handleConnection(CivetServer *server, const struct mg_connection *conn) {
    
    std::cout<<"WS connected\n";
    return true;

  
}

void WebSocketHandler::handleReadyState(CivetServer *server, struct mg_connection *conn) {
   
    std::cout<<"WS ready\n";
    const char *text = "Hello from XBotCore";
    mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, text, strlen(text));
    
    buffer->increaseNumClient();
    
} 
        
bool WebSocketHandler::handleData(CivetServer *server,
                                struct mg_connection *conn,
                                int bits,
                                char *data,
                                size_t data_len) {
                
    //read robot state
    std::vector<double> vec;
    bool resp = buffer->remove(vec);
        
    StringBuffer buffer;
    
    //vec.serialize(buffer);
    
    Writer<StringBuffer> writer(buffer);
    const char* btosend = nullptr;
    
    //send robot state
    buffer.Clear();
    if(resp){
      
        writer.StartObject();              
        writer.Key("link_position");   
        writer.StartArray();
        for( double val : vec ){  
          writer.Double(val);
        }
        
        writer.EndArray();
        writer.EndObject();  
        
        btosend = buffer.GetString();
    }
    
    
    if( btosend!=nullptr)
      mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, btosend, buffer.GetLength());

        
    return true;
}  

void WebSocketHandler::handleClose(CivetServer *server, const struct mg_connection *conn) {
       std::cout<<"WS closed\n";
       buffer->decreaseNumClient();
}        
        
CommunicationInterfaceWebServer::CommunicationInterfaceWebServer():
    CommunicationInterface()
{
    
}

CommunicationInterfaceWebServer::CommunicationInterfaceWebServer(XBotInterface::Ptr robot):
    CommunicationInterface(robot),
    _path_to_cfg(robot->getPathToConfig())
{
    const char *options[] = {
            "document_root", DOCUMENT_ROOT, "listening_ports", PORT, 0};
    
    std::vector<std::string> cpp_options;
    for (int i=0; i<(sizeof(options)/sizeof(options[0])-1); i++) {
        cpp_options.push_back(options[i]);
    }      
    
    
    buffer = std::make_shared<Buffer>();
    server = std::make_shared<CivetServer>(cpp_options);  
    ws_handler = std::make_shared<WebSocketHandler>();
    ws_handler->buffer = buffer;
    server->addWebSocketHandler("/websocket", *ws_handler);
  
    //creo oggetto che implementa interfaccia e lo passo all hanlder
    std::cout<<"XBotCore server running at http://"<<PORT<<std::endl;        
}


void CommunicationInterfaceWebServer::sendRobotState()
{

  if(buffer->getNumClient().load() <= 0) return;
  
  //read from robot
  //write to a buffer that the callback handleData will use
  _robot->getJointPosition(_joint_id_map);
  //TBD create object that encapsulate all robot info and push it to the buffer
  //std::vector<XBot::RobotState::pdo_tx> rstate;
  
  std::vector<double> vec;
  for( int id : _robot->getEnabledJointId() ){       
      double val= _joint_id_map.at(id);
      vec.push_back(val);
      //XBot::RoboState::pdo_tx tmp;
      //tmp.pos_ref = val;
      //rstate.push_back(XBot::RoboState::pdo_tx
  }

  buffer->add(vec);
  
}

void CommunicationInterfaceWebServer::receiveReference()
{
    //use buffer
    //TODO in the case of hololens (we just send position goal), so the client makes a GET
  //and we save in a thread safe structure that this method will use
  
  //std::cout<<"RECEIVE WEB"<<std::endl;
 
}

bool CommunicationInterfaceWebServer::advertiseSwitch(const std::string& port_name)
{
    http_handler = std::make_shared<HttpHandler>(*this, buffer);
    s_handler = std::make_shared<HttpCivetHandler>(*http_handler);
    server->addHandler(SWITCH_URI, *s_handler);
    server->addHandler(CMD_URI, *s_handler);
    server->addHandler(MASTER_URI, *s_handler);
    _switch[port_name] = "";

    return true;
}

void XBot::CommunicationInterfaceWebServer::advertiseStatus(const std::string& plugin_name)
{
     _status[plugin_name] = "";

}


bool XBot::CommunicationInterfaceWebServer::setPluginStatus(const std::string& plugin_name, const std::string& status)
{
    _status[plugin_name] = status;
    return true;
}


bool XBot::CommunicationInterfaceWebServer::advertiseCmd(const std::string& port_name)
{
    _cmd[port_name] = "";
    return true;
}


bool XBot::CommunicationInterfaceWebServer::advertiseMasterCommunicationInterface()
{
    return true;
}



bool CommunicationInterfaceWebServer::receiveFromSwitch(const std::string& port_name, std::string& message)
{
    message = _switch[port_name];
    _switch[port_name] = "";
    if (message.compare("")==0) return false;
    return true;
    
}

bool XBot::CommunicationInterfaceWebServer::receiveFromCmd(const std::string& port_name, std::string& message)
{
    
    message = _cmd[port_name];
    _cmd[port_name] = "";
    if (message.compare("")==0) return false;
    return true;
    
}

bool XBot::CommunicationInterfaceWebServer::receiveMasterCommunicationInterface(std::string& framework_name)
{ 
    std::string master = "";
    buffer->getMaster(framework_name);
    buffer->setMaster(master);
    return true;
   
}



bool CommunicationInterfaceWebServer::computeAbsolutePath (  const std::string& input_path,
                                                 const std::string& middle_path,
                                                 std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}


}