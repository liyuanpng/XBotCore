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
    
    server = std::make_shared<CivetServer>(cpp_options);  
    ws_handler = std::make_shared<WebSocketHandler>();
    server->addWebSocketHandler("/websocket", *ws_handler);
  
    std::cout<<"XBotCore server running at http://"<<PORT<<SWITCH_URI<<std::endl;        
}


void CommunicationInterfaceWebServer::sendRobotState()
{

  //use websocket
  //try to send json over pipe
   
   
}

void CommunicationInterfaceWebServer::receiveReference()
{
    //use websocket
  //try to send json over pipe
}

bool CommunicationInterfaceWebServer::advertiseSwitch(const std::string& port_name)
{
   
    s_handler = std::make_shared<SwitchHandler>(_status,_switch,_cmd);
    server->addHandler(SWITCH_URI, *s_handler);
    server->addHandler(CMD_URI, *s_handler);
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
  
    return false;
   
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