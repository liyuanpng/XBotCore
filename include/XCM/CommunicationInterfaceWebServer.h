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
#include <HttpCivetHandler.h>
#include <WebSocketCivetHandler.h>

//192.168.0.100
#define DOCUMENT_ROOT "./pages"
//#define PORT "192.168.0.100:8081"
//#define PORT "10.255.32.80:8081"
#define PORT "127.0.0.1:8081"
#define SWITCH_URI "/switch"
#define CMD_URI "/cmd"
#define ALLJOINT_URI "/alljoints"
#define SINGLEJOINT_URI "/singlejoint"
#define MASTER_URI "/webmaster"

namespace XBot {

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

}

#endif //__XBOT_COMMUNICATION_INTERFACE_WEBSERVER_H__
