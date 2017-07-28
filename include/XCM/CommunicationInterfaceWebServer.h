/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore, Giuseppe Rigano
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it, giuseppe.rigano@iit.it
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

//192.168.0.100
#define DOCUMENT_ROOT "./pages"
//#define PORT "192.168.0.100:8081"
#define PORT "127.0.0.1:8081"
#define SWITCH_URI "/switch"
#define CMD_URI "/cmd"

namespace XBot {

class SwitchHandler : public CivetHandler
{
  
  private:
    
    std::map<std::string,std::string>* status_map;
    std::map<std::string,std::string>* switch_map;
    std::map<std::string,std::string>* cmd_map;
 
  public:
          
    SwitchHandler(std::map<std::string,std::string> &map,std::map<std::string,std::string> &switch_map, std::map<std::string,std::string> &cmd_map){
      
      this->status_map = &map;
      this->switch_map = &switch_map;
      this->cmd_map = &cmd_map;
    } 
      
    
    bool handleGet(CivetServer *server, struct mg_connection *conn) {
      
      const char* uri =mg_get_request_info(conn)->request_uri;
      std::string suri(uri);
//       std::cout<<" "<<suri<<std::endl;
      const char* query = mg_get_request_info(conn)->query_string;
      if (query!=nullptr){
      std::string squery(query);
      std::string key = squery.substr(0, squery.find("="));
      std::string val = squery.substr(squery.find("=")+1);
      if(suri.compare("/switch")==0){
	auto& m= *switch_map;
	m[key]=val; 
      }
      else if(suri.compare("/cmd")==0) {
	auto& m= *cmd_map;
	m[key]=val; 
      }
      }
      
      
      
      mg_printf(conn,
              "HTTP/1.1 200 OK\r\nContent-Type: "
              "text/html\r\nConnection: close\r\n\r\n");
      mg_printf(conn, "<html><body>\r\n");
      mg_printf(conn,
              "<h2>XBOTCORE </h2>\r\n");
      for( auto const &s : *status_map){                 
        auto const &outer_key = s.first;
        auto const &inner_map = s.second;
        std::string ss="<h3>"+ outer_key+ " "+ inner_map+ "</h3>\r\n";
        const char * w =const_cast<char*>( ss.c_str());

        mg_printf(conn, w);

      }



      mg_printf(conn, "</body></html>\r\n");
            return true;
    }

};

// class CmdHandler : public CivetHandler
// {
//   
//   private:
//     std::map<std::string,std::string>* cmd_map;
//  
//   public:
//           
//     CmdHandler(std::map<std::string,std::string> &map){
//       
//       this->cmd_map = &map;
//      
//     } 
//       
//     
//     bool handleGet(CivetServer *server, struct mg_connection *conn) {
//       
//       const char* uri =mg_get_request_info(conn)->request_uri;
//       std::cout<<" "<<uri<<std::endl;
//       const char* query = mg_get_request_info(conn)->query_string;
//       if (query!=nullptr){
//       std::string squery(query);
//       std::string key = squery.substr(0, squery.find("="));
//       std::string val = squery.substr(squery.find("=")+1);
//       auto& m= *cmd_map;
//       m[key]=val;          
//       }
//       
//       
//       
//       mg_printf(conn,
//               "HTTP/1.1 200 OK\r\nContent-Type: "
//               "text/html\r\nConnection: close\r\n\r\n");
//       mg_printf(conn, "<html><body>\r\n");
//       mg_printf(conn,
//               "<h2>XBOTCORE </h2>\r\n");
//       for( auto const &s : *status_map){                 
//         auto const &outer_key = s.first;
//         auto const &inner_map = s.second;
//         std::string ss="<h3>"+ outer_key+ " "+ inner_map+ "</h3>\r\n";
//         char * w =const_cast<char*>( ss.c_str());
// 
//         mg_printf(conn, w);
// 
//       }
// 
// 
// 
//       mg_printf(conn, "</body></html>\r\n");
//             return true;
//     }
// 
// };

  
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

};


}

#endif //__XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
