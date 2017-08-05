/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
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


#ifndef __XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
#define __XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__

#include <XCM/XBotCommunicationInterface.h>
#include <XBotCore-interfaces/XBotESC.h>
#include "CivetServer.h"
#include <boost/bind.hpp>

#include <boost/circular_buffer.hpp>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;


//192.168.0.100
#define DOCUMENT_ROOT "./pages"
//#define PORT "192.168.0.100:8081"
#define PORT "127.0.0.1:8081"
#define SWITCH_URI "/switch"
#define CMD_URI "/cmd"
#define MASTER_URI "/webmaster"

namespace XBot {
class Buffer;
class HttpInterface;

class ResponseInterface {
  
  public:
    virtual const void * GetData() = 0;
    
    virtual std::size_t GetLength() = 0;
    
    virtual std::string GetTypeResponse() = 0;
    
};

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


class HttpCivetHandler : public CivetHandler
{
  
  private:
    
    HttpInterface* http_interface;
  
  public:
    
    HttpCivetHandler(HttpInterface& interface){
      
      http_interface = &interface;
     
    }  
    
    bool handleGet(CivetServer *server, struct mg_connection *conn);
};


class Buffer {
   
  public:
  
    void add(std::vector<double>& vec) {       
            std::lock_guard<std::mutex> locker(mutex);  
            circular_buffer.push_back(vec);
            return;       
    }
    
    
    bool remove(std::vector<double>& vec) {
      
            if(!circular_buffer.empty()){
              std::lock_guard<std::mutex> locker(mutex);            
              std::vector<double> back = circular_buffer.front();
              circular_buffer.pop_front();
              vec = back;
              return true;
            }
        return false;
    }
    
    
    void clear() {
      
            if(!circular_buffer.empty()){
              std::lock_guard<std::mutex> locker(mutex); 
              //boost::circular_buffer<std::vector<double>> empty;
              circular_buffer.clear();
              //circular_buffer.swap(empty);
            }
        return;
    }
    
    Buffer() {
      
      num_client.store(0);
      master = "";
      circular_buffer.set_capacity(100);
      
    }
    
    void increaseNumClient(){      
      num_client++;
      std::cout<<"Num_Client "<< num_client.load()<<std::endl;
    }
    
    void decreaseNumClient(){     
      num_client--;
      std::cout<<"Num_Client "<< num_client.load()<<std::endl;      
    }
    
    std::atomic<int>& getNumClient(){
      return num_client;
    }
    
     
    void setMaster(std::string& val) {       
            std::lock_guard<std::mutex> locker(m_master);  
            master = val;
            return;       
    }
    
    void getMaster(std::string& val) {       
            std::lock_guard<std::mutex> locker(m_master);  
            val = master;
            return;       
    }
    
  private:
       
    std::mutex mutex;
    std::atomic<int> num_client;
    
    boost::circular_buffer<std::vector<double>> circular_buffer;
    std::mutex m_master;
    std::string master;

};

class CommunicationInterfaceWebServer;
class WebSocketHandler : public CivetWebSocketHandler {

  public:
  
    std::shared_ptr<Buffer> buffer;
        
    virtual bool handleConnection(CivetServer *server, const struct mg_connection *conn);

    virtual void handleReadyState(CivetServer *server, struct mg_connection *conn);

    virtual bool handleData(CivetServer *server,
                            struct mg_connection *conn,
                            int bits,
                            char *data,
                            size_t data_len);

    virtual void handleClose(CivetServer *server, const struct mg_connection *conn);

};

class HttpInterface {
  
  public:
    
      virtual void handleGet(std::shared_ptr<ResponseInterface>& response) = 0;
      
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



class HttpHandler;
class CommunicationInterfaceWebServer : public CommunicationInterface {

  public:

      CommunicationInterfaceWebServer();
      CommunicationInterfaceWebServer(XBotInterface::Ptr robot);

      virtual void sendRobotState();
      virtual void receiveReference();

      virtual bool advertiseSwitch(const std::string& port_name);
      virtual bool receiveFromSwitch(const std::string& port_name, std::string& message);

      virtual bool advertiseCmd(const std::string& port_name);
      virtual bool receiveFromCmd(const std::string& port_name, std::string& message);  // TBD template message

      virtual bool advertiseMasterCommunicationInterface();
      virtual bool receiveMasterCommunicationInterface(std::string& framework_name);

      virtual void advertiseStatus(const std::string& plugin_name);
      virtual bool setPluginStatus(const std::string& plugin_name, const std::string& status);
      
      
      std::map<std::string, std::string>& getSwitchMap(){
	
	return _switch;
      }
      
      std::map<std::string, std::string>& getStatusMap(){
	
	return _status;
      }
      
      std::map<std::string, std::string>& getCmdMap(){
	
	return _cmd;
      }

  protected:

  private:

    std::shared_ptr<CivetServer> server;
    std::shared_ptr<HttpCivetHandler> s_handler;
    std::shared_ptr<WebSocketHandler> ws_handler;
    
    std::shared_ptr<HttpHandler> http_handler;
      

    static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& middle_path,
                                       std::string& absolute_path );

    bool _send_robot_state_ok, _receive_commands_ok;

    std::string _path_to_cfg;

    JointIdMap _joint_id_map;
    JointNameMap _joint_name_map;

  
    std::map<std::string, std::string> _plugin_status_map;
   

    std::unordered_map<int, int> _jointid_to_command_msg_idx;
    std::unordered_map<int, int> _jointid_to_jointstate_msg_idx;

   
    std::string _tf_prefix, _urdf_param_name;

   
    std::map<int, double> _hand_value_map;
    
    
    //TODO to put in a shared object
    std::map<std::string, std::string> _switch;
    std::map<std::string, std::string> _status;
    std::map<std::string, std::string> _cmd;
    
    std::shared_ptr<Buffer> buffer;

};


class HttpHandler : public HttpInterface{
  
  private:
    
    CommunicationInterface* comm; 
    std::shared_ptr<Buffer> buffer;
  
  public:
    
    HttpHandler (CommunicationInterface& comm, std::shared_ptr<Buffer> buffer){
      
	this->comm = &comm;
	this->buffer = buffer;
    }
    
  
    virtual void handleGet(std::shared_ptr<ResponseInterface>& response){
      
      CommunicationInterfaceWebServer* comm_web = reinterpret_cast<CommunicationInterfaceWebServer*>(this->comm);
    
    
      if(uri.compare("/switch")==0){
        auto& m= comm_web->getSwitchMap();
        m[key]=val; 
      }
      else if(uri.compare("/cmd")==0) {
        auto& m= comm_web->getCmdMap();
        m[key]=val; 
      }
       
      else if(uri.compare("/webmaster")==0){
        buffer->setMaster(key);
      }
      
      
     
//       std::string sresp="";
//       sresp+="<html><body>\r\n";
//       sresp+=("<h2>XBOTCORE </h2>\r\n");
//       
//       for( auto const &s : comm_web->getStatusMap()){                 
// 	  auto const &outer_key = s.first;
// 	  auto const &inner_map = s.second;
// 	  std::string ss="<h3>"+ outer_key+ " "+ inner_map+ "</h3>\r\n";
// 	  const char * w =const_cast<char*>( ss.c_str());
// 	  sresp+=(w);
//       }
// 
//       sresp+=("</body></html>\r\n");
// 
//       response = std::make_shared<StringResponse>(sresp);
     
      std::shared_ptr<StringBuffer> jsonresp = std::make_shared<StringBuffer>();
      // 1. Parse a JSON string into DOM.
      const char* json = "[{\"project\":\"rapidjson\",\"stars\":10}, {\"project\":\"rapidjson\",\"stars\":10}]";
      Document d;
      d.Parse(json);

      // 2. Modify it by DOM.
      Value& s = d["stars"];
      s.SetInt(s.GetInt() + 1);

      // 3. Stringify the DOM
      Writer<StringBuffer> writer(*jsonresp);
      d.Accept(writer);
      
      response = std::make_shared<JsonResponse>(jsonresp);
     
             
  }
  
};

}

#endif //__XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
