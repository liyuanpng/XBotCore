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
#define ALLJOINT_URI "/alljoints"
#define SINGLEJOINT_URI "/singlejoint"
#define MASTER_URI "/webmaster"

namespace XBot {
template <class T>
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

class RequestObject {
  
private:
    
    void *buff;
  
  public:
    
    void SetData(void* buff) {
      
        this->buff = buff;
    }
    
    void* GetData() {
     
        return this->buff;
        
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
    bool handlePost(CivetServer* server, mg_connection* conn);
};

template <class T>
class Buffer {
   
  public:
  
    Buffer(int capacity) {
      circular_buffer.set_capacity(capacity);
    }
    
    void add(T& vec) {       
        std::lock_guard<std::mutex> locker(mutex);  
        circular_buffer.push_back(vec);
        return;       
    }
        
    bool remove(T& vec) {
      
        if(!circular_buffer.empty()){
            std::lock_guard<std::mutex> locker(mutex);            
            T back = circular_buffer.front();
            circular_buffer.pop_front();
            vec = back;
            return true;
        }
        
        return false;
    }    
    
    void clear() {
      
        if(!circular_buffer.empty()){
            std::lock_guard<std::mutex> locker(mutex);             
            circular_buffer.clear();            
        }
        return;
    }
       
    
        
  private:
       
    std::mutex mutex;    
    boost::circular_buffer<T> circular_buffer;   

};

class SharedData {
   
  public:
  
     SharedData():master("") { 
       num_client.store(0);
       external_command = std::make_shared<Buffer<std::vector<double>>>(5);
    }    
     
     std::map<std::string, std::string> getAllStatus(){
       std::lock_guard<std::mutex> locker(st_mutex); 
       return _status;
    }
      
    void insertSwitch(std::string key, std::string val){
        std::lock_guard<std::mutex> locker(s_mutex); 
        _switch[key] = val;
    }
    
    std::string getSwitch(std::string key){
        std::lock_guard<std::mutex> locker(s_mutex); 
        return _switch[key];
    }

    void insertCmd(std::string key, std::string val){
        std::lock_guard<std::mutex> locker(c_mutex); 
        _cmd[key] = val;
    }
    
    std::string getCmd(std::string key){
        std::lock_guard<std::mutex> locker(c_mutex); 
        return _cmd[key];
    }

    void insertStatus(std::string key, std::string val){
        std::lock_guard<std::mutex> locker(st_mutex); 
        _status[key] = val;
    }
    
    std::string getStatus(std::string key){
        std::lock_guard<std::mutex> locker(st_mutex); 
        return _status[key];
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
    
    std::shared_ptr<Buffer<std::vector<double>>> external_command;
    
  private:
       
    std::mutex m_master;
    std::string master;
    std::atomic<int> num_client; 
    
    std::map<std::string, std::string> _switch;
    std::mutex s_mutex;
    std::map<std::string, std::string> _status;
    std::mutex st_mutex;
    std::map<std::string, std::string> _cmd;
    std::mutex c_mutex;
   
};

class WebRobotState {
  
  public:
    
    std::vector<std::string> joint_name;
    std::vector<int> joint_id;
    std::vector<double> link_position;
    std::vector<double> motor_position;
    std::vector<double> link_vel;
    std::vector<double> motor_vel;
    std::vector<double> temperature;
    
    //XBot::RobotState::pdo_rx pdo_rx;
    //IMU
    //FT
  
    void serialize(StringBuffer& buffer){
      
        Writer<StringBuffer> writer(buffer);
        writer.StartObject();  
        serializeArray(writer,"joint_id",joint_id);
        serializeArray(writer,"link_position",link_position);
        serializeArray(writer,"motor_position", motor_position);
        serializeArray(writer,"link_velocity",link_vel);
        serializeArray(writer,"motor_velocity",motor_vel);
        serializeArray(writer,"temperature",temperature);      
        writer.EndObject();  
    }
    
  private:
    
    //template <typename T>
    void serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<double>& array){
      
       // writer.StartObject();              
        writer.Key(key.c_str());   
        writer.StartArray();
        for( double val : array ){  
          writer.Double(val);
        }

        writer.EndArray();
       // writer.EndObject();  
    }
    
    void serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<int>& array){
      
       // writer.StartObject();              
        writer.Key(key.c_str());   
        writer.StartArray();
        for( int val : array ){  
          writer.Int(val);
        }

        writer.EndArray();
       // writer.EndObject();  
    }
  
};

class CommunicationInterfaceWebServer;
class WebSocketHandler : public CivetWebSocketHandler {

  public:
    
    WebSocketHandler(std::shared_ptr<Buffer<WebRobotState>> buffer, std::shared_ptr<SharedData> sharedData){
      
      this->buffer = buffer;
      this->sharedData = sharedData;      
    }
           
    virtual bool handleConnection(CivetServer *server, const struct mg_connection *conn);

    virtual void handleReadyState(CivetServer *server, struct mg_connection *conn);

    virtual bool handleData(CivetServer *server,
                            struct mg_connection *conn,
                            int bits,
                            char *data,
                            size_t data_len);

    virtual void handleClose(CivetServer *server, const struct mg_connection *conn);

  private:
    std::shared_ptr<Buffer<WebRobotState>> buffer;
    std::shared_ptr<SharedData> sharedData;
};

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
      virtual bool receiveFromCmd(const std::string& port_name, std::string& message);  

      virtual bool advertiseMasterCommunicationInterface();
      virtual bool receiveMasterCommunicationInterface(std::string& framework_name);

      virtual void advertiseStatus(const std::string& plugin_name);
      virtual bool setPluginStatus(const std::string& plugin_name, const std::string& status);      

  private:

      std::shared_ptr<CivetServer> server;
      std::shared_ptr<HttpCivetHandler> http_civet_handler;
      std::shared_ptr<WebSocketHandler> ws_civet_handler;    
      std::shared_ptr<HttpHandler> http_handler;
        

      static bool computeAbsolutePath ( const std::string& input_path,
                                        const std::string& middle_path,
                                        std::string& absolute_path );

      bool _send_robot_state_ok, _receive_commands_ok;

      std::string _path_to_cfg;

      JointIdMap _joint_id_map;
      JointNameMap _joint_name_map;

      std::unordered_map<int, int> _jointid_to_command_msg_idx;
      std::unordered_map<int, int> _jointid_to_jointstate_msg_idx;

    
      std::string _tf_prefix, _urdf_param_name;   
      std::map<int, double> _hand_value_map;
      
      std::shared_ptr<Buffer<WebRobotState>> buffer;
      std::shared_ptr<SharedData> sharedData;
    
};


class HttpHandler : public HttpInterface{
  
  private:
    
    std::shared_ptr<SharedData> sharedData; 
    std::shared_ptr<Buffer<WebRobotState>> buffer;
  
  public:
    
    HttpHandler (std::shared_ptr<SharedData>& sharedData, std::shared_ptr<Buffer<WebRobotState>>& buffer){
      
	this->sharedData = sharedData;
	this->buffer = buffer;
    }
    
  
    virtual void handleGet(std::shared_ptr<ResponseInterface>& response){      
         
      if(uri.compare("/switch")==0){
        sharedData->insertSwitch(key, val);       
      }
      else if(uri.compare("/cmd")==0) {
        sharedData->insertCmd(key, val);        
      }       
      else if(uri.compare("/webmaster")==0){
        sharedData->setMaster(key);
      }
      
      std::string sresp="";
      sresp+="<html><body>\r\n";
      sresp+=("<h2>XBOTCORE </h2>\r\n");
      
      for( auto const &s : sharedData->getAllStatus()){                 
	  auto const &outer_key = s.first;
	  auto const &inner_map = s.second;
	  std::string ss="<h3>"+ outer_key+ " "+ inner_map+ "</h3>\r\n";
	  const char * w =const_cast<char*>( ss.c_str());
	  sresp+=(w);
      }

      sresp+=("</body></html>\r\n");

      response = std::make_shared<StringResponse>(sresp);
     
//       std::shared_ptr<StringBuffer> jsonresp = std::make_shared<StringBuffer>();
//       // 1. Parse a JSON string into DOM.
//       const char* json = "[{\"project\":\"rapidjson\",\"stars\":10}, {\"project\":\"rapidjson\",\"stars\":10}]";
//       Document d;
//       d.Parse(json);
// 
//       // 2. Modify it by DOM.
//       Value& s = d["stars"];
//       s.SetInt(s.GetInt() + 1);
// 
//       // 3. Stringify the DOM
//       Writer<StringBuffer> writer(*jsonresp);
//       d.Accept(writer);
//       
//       response = std::make_shared<JsonResponse>(jsonresp);
     
             
  }
  
  virtual void handlePost(std::shared_ptr<RequestObject>& request){
    
      void * buff;
      buff = request->GetData();     
      StringStream stream((char*)buff);
      //std::cout<<"pos"<<std::string((char*)buff)<<std::endl;
      Document d;
      d.ParseStream(stream);
      
      std::vector<double> vec;
      if(uri.compare("/alljoints")==0){     
        if( d.HasMember("link_position")){        
          assert(d["link_position"].isArray());
          const Value& array = d["link_position"];
          for (SizeType i = 0; i < array.Size(); i++){
              double val = array[i].GetDouble();
              vec.push_back(val);   
          }
          sharedData->external_command->add(vec);
          //HACK simulation of holding value for longer time
          sharedData->external_command->add(vec);
        }
      }else if(uri.compare("/singlejoint")==0){
        
        //add value to map(id,joint) in shared data
        
        
      }
            
      /*StringBuffer buffer;
      Writer<StringBuffer> writer(buffer);
      d.Accept(writer);
      std::cout <<"stringify"<< std::string(buffer.GetString()) << std::endl;  */ 
    
  }
  
};

}

#endif //__XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
