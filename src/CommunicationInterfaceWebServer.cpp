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
      std::string type = resp_interface->GetTypeResponse();
      std::string header = "HTTP/1.1 200 OK\r\nContent-Type: "
			    +type+"\r\nConnection: close\r\n\r\n";

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
      req->SetData(buf);
      http_interface->handlePost(req);
      
      mg_printf(conn,"{\"response\":\"ok\"}");
  
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
    
    buffer = std::make_shared<Buffer<WebRobotState>>(50);    
    sharedData = std::make_shared<SharedData>();
    server = std::make_shared<CivetServer>(cpp_options);  
    ws_civet_handler = std::make_shared<WebSocketHandler>(buffer, sharedData);   
    server->addWebSocketHandler("/websocket", *ws_civet_handler);  
    
    std::cout<<"XBotCore server running at http://"<<PORT<<std::endl;        
}

void CommunicationInterfaceWebServer::sendRobotState()
{

    if(sharedData->getNumClient().load() <= 0) return;
    
    //read from robot
    //write to a buffer that the callback handleData will use
    JointIdMap _joint_id_map, _motor_id_map,
    _jvel_id_map,_mvel_id_map, _temp_id_map;
    
    _robot->getJointPosition(_joint_id_map);
    _robot->getMotorPosition(_motor_id_map);
    _robot->getJointVelocity(_jvel_id_map);
    _robot->getMotorVelocity(_mvel_id_map);
    _robot->getTemperature(_temp_id_map);
    
    WebRobotState rstate;  
    
    for ( auto s: _robot->getEnabledJointNames()) {      
        rstate.joint_name.push_back(s);      
    }
    
    for( int id : _robot->getEnabledJointId() ){       
        double jval= _joint_id_map.at(id);
        double mval= _motor_id_map.at(id);
        double jvelval= _jvel_id_map.at(id);
        double mvelval= _mvel_id_map.at(id);
        double tempval= _temp_id_map.at(id);
                 
        rstate.joint_id.push_back(id);
        rstate.link_position.push_back(jval);
        rstate.motor_position.push_back(mval);
        rstate.link_vel.push_back(jvelval);
        rstate.motor_vel.push_back(mvelval); 
        rstate.temperature.push_back(tempval);        
    }
    
    buffer->add(rstate);   
}

void CommunicationInterfaceWebServer::receiveReference()
{
    std::vector<double> vec;
    bool resp = sharedData->external_command->remove(vec);    
    
    if (resp){ 
      Eigen::VectorXd eigVec;
      eigVec.resize(31);
      for (int i=0; i< vec.size(); i++){
         eigVec(i) = vec[i];
      }     
      _robot->setPositionReference(eigVec);
    }
    
      //set single joint value      
      JointIdMap tmp;
      _robot->getPositionReference(tmp);
      std::map<int,double> map = sharedData->getJointMap();
      for( auto& m : map){
        int id = m.first;
        double val = m.second;
        tmp.at(id)= val;      
      } 
      if(!map.empty())
        _robot->setPositionReference(tmp);  
    
}

bool CommunicationInterfaceWebServer::advertiseSwitch(const std::string& port_name)
{
    http_handler = std::make_shared<HttpHandler>(sharedData, buffer);
    http_civet_handler = std::make_shared<HttpCivetHandler>(*http_handler);
    server->addHandler(SWITCH_URI, *http_civet_handler);
    server->addHandler(CMD_URI, *http_civet_handler);
    server->addHandler(MASTER_URI, *http_civet_handler);
    server->addHandler(ALLJOINT_URI, *http_civet_handler);
    server->addHandler(SINGLEJOINT_URI, *http_civet_handler);
    sharedData->insertSwitch(port_name, "");
    
    return true;
}

void XBot::CommunicationInterfaceWebServer::advertiseStatus(const std::string& plugin_name)
{
    sharedData->insertStatus(plugin_name, "");
}

bool XBot::CommunicationInterfaceWebServer::setPluginStatus(const std::string& plugin_name, const std::string& status)
{
    sharedData->insertStatus(plugin_name, status);
    return true;
}

bool XBot::CommunicationInterfaceWebServer::advertiseCmd(const std::string& port_name)
{
    sharedData->insertCmd(port_name, "");
    return true;
}

bool XBot::CommunicationInterfaceWebServer::advertiseMasterCommunicationInterface()
{
    return true;
}

bool CommunicationInterfaceWebServer::receiveFromSwitch(const std::string& port_name, std::string& message)
{
    message = sharedData->getSwitch(port_name);
    sharedData->insertSwitch(port_name, "");
    if (message.compare("")==0) return false;
    return true;    
}

bool XBot::CommunicationInterfaceWebServer::receiveFromCmd(const std::string& port_name, std::string& message)
{
    
    message = sharedData->getCmd(port_name);   
    sharedData->insertCmd(port_name, "");
    if (message.compare("")==0) return false;
    return true;    
}

bool XBot::CommunicationInterfaceWebServer::receiveMasterCommunicationInterface(std::string& framework_name)
{ 
    std::string master = "";
    sharedData->getMaster(framework_name);
    sharedData->setMaster(master);
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