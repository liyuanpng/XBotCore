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

namespace XBot {
class Buffer;
class SwitchHandler : public CivetHandler
{
  
  private:
    
    std::map<std::string,std::string>* status_map;
    std::map<std::string,std::string>* switch_map;
    std::map<std::string,std::string>* cmd_map;
    
    std::shared_ptr<Buffer> buffer;
 
  public:
          
    SwitchHandler(std::map<std::string,std::string> &map,std::map<std::string,std::string> &switch_map, std::map<std::string,std::string> &cmd_map, std::shared_ptr<Buffer> buffer){
      
      this->status_map = &map;
      this->switch_map = &switch_map;
      this->cmd_map = &cmd_map;
      this->buffer = buffer;
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
    
    
  private:
       
    std::mutex mutex;
    std::atomic<int> num_client;
    boost::circular_buffer<std::vector<double>> circular_buffer;

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

  protected:

  private:

    std::shared_ptr<CivetServer> server;
    std::shared_ptr<SwitchHandler> s_handler;
    std::shared_ptr<WebSocketHandler> ws_handler;
    
      

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
    
    std::map<std::string, std::string> _switch;
    std::map<std::string, std::string> _status;
    std::map<std::string, std::string> _cmd;
    
    std::shared_ptr<Buffer> buffer;

};


}

#endif //__XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
