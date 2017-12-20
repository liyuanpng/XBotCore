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

extern "C" XBot::CommunicationInterfaceWebServer* create_instance(XBot::RobotInterface::Ptr robot)
{
  return new XBot::CommunicationInterfaceWebServer(robot);
}

extern "C" void destroy_instance( XBot::CommunicationInterfaceWebServer* instance )
{
  delete instance;
}

namespace XBot {
              
CommunicationInterfaceWebServer::CommunicationInterfaceWebServer():
    CommunicationInterface()
{
    
}

CommunicationInterfaceWebServer::CommunicationInterfaceWebServer(XBotInterface::Ptr robot):
    CommunicationInterface(robot),
    _path_to_cfg(robot->getPathToConfig())
{
    std::string aport = PORT;
    YAML::Node root_cfg = YAML::LoadFile(_path_to_cfg);
    const YAML::Node &web_server = root_cfg["WebServer"]; 
    if(web_server){
       address = web_server["address"].as<std::string>();
       port = web_server["port"].as<std::string>();
       aport = address +":"+port;
    } 
    
    std::string droot = "";
    const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
    if( env_p != nullptr && !(strcmp(env_p,"") == 0))
       droot = std::string(env_p) + std::string(DOCUMENT_ROOT);
    
    const char *options[] = {
            "document_root", droot.c_str(), "listening_ports", aport.c_str(), 0};
    
    std::vector<std::string> cpp_options;
    for (int i=0; i<(sizeof(options)/sizeof(options[0])-1); i++) {
        cpp_options.push_back(options[i]);
    }      
    
    numjoint = _robot->getJointNum();
    buffer = std::make_shared<Buffer<WebRobotStateTX>>(50);    
    sharedData = std::make_shared<SharedData>();
    try{
      server = std::make_shared<CivetServer>(cpp_options);  
    }catch( CivetException e ){ std::cout<<"Port "<< aport <<" already in use from another process"<<std::endl; std::cout<<e.what()<<std::endl; exit(1);}
    ws_civet_handler = std::make_shared<WebSocketHandler>(buffer, sharedData);   
    server->addWebSocketHandler("/websocket", *ws_civet_handler);  
    for ( auto &chainmap : _robot->getChainMap()){
        std::string key =chainmap.first;
        XBot::KinematicChain::Ptr chain = chainmap.second;
        std::vector<std::string> ids;
        //populate ids
        for( int i=0; i<chain->getJointIds().size();i ++){
          ids.push_back(std::to_string(chain->getJointIds()[i]));
        }
        
        std::vector<std::string> names = chain->getJointNames();
        std::vector<std::string> jvals, vref, eref, stiff, damp;
        //populate jvals
        
        std::vector <std::string> lowlimit;
        std::vector <std::string> uplimit;
        for( int i=0; i<ids.size();i ++){
          jvals.push_back(std::to_string(chain->getJointPosition(i)));
          vref.push_back(std::to_string(chain->getJointVelocity(i)));
          eref.push_back(std::to_string(chain->getJointEffort(i)));
          stiff.push_back(std::to_string(chain->getStiffness(i)));
          damp.push_back(std::to_string(chain->getDamping(i)));
          double llimit, ulimit;
          chain->getJointLimits(i,llimit,ulimit);
          lowlimit.push_back(std::to_string(llimit));
          uplimit.push_back(std::to_string(ulimit));
        }
        
        std::vector < std::vector<std::string> > val;
        //populate val
        val.push_back(ids);
        val.push_back(names);
        val.push_back(jvals);
        val.push_back(vref);
        val.push_back(eref);
        val.push_back(stiff);
        val.push_back(damp);
        val.push_back(lowlimit);
        val.push_back(uplimit);
        
        sharedData->insertChain(key,val);
    }
    
    Logger::info(Logger::Severity::MID) << "XBotCore server running at http://" << aport << Logger::endl();
}

void CommunicationInterfaceWebServer::sendRobotState()
{

    //read from robot
    //write to a buffer that the callback handleData will use
    JointIdMap _joint_id_map, _motor_id_map,
    _jvel_id_map,_mvel_id_map, _temp_id_map,
    _effort_id_map, _stiffnes_id_map,_damping_id_map;
    
    _robot->getJointPosition(_joint_id_map);
    _robot->getMotorPosition(_motor_id_map);
    _robot->getJointVelocity(_jvel_id_map);
    _robot->getMotorVelocity(_mvel_id_map);
    _robot->getTemperature(_temp_id_map);
    _robot->getJointEffort(_effort_id_map);
    _robot->getStiffness(_stiffnes_id_map);
    _robot->getDamping(_damping_id_map);
    
    WebRobotStateTX rstate;  
    
    for ( auto s: _robot->getEnabledJointNames()) {      
        rstate.joint_name.push_back(s);      
    }
    
    for( int id : _robot->getEnabledJointId() ){       
        double jval= _joint_id_map.at(id);
        double mval= _motor_id_map.at(id);
        double jvelval= _jvel_id_map.at(id);
        double mvelval= _mvel_id_map.at(id);
        double tempval= _temp_id_map.at(id);
        double effval= _effort_id_map.at(id);
        double stiffval= _stiffnes_id_map.at(id);
        double dampval= _damping_id_map.at(id);
                 
        rstate.joint_id.push_back(id);
        rstate.link_position.push_back(jval);
        rstate.motor_position.push_back(mval);
        rstate.link_vel.push_back(jvelval);
        rstate.motor_vel.push_back(mvelval); 
        rstate.temperature.push_back(tempval); 
        rstate.effort.push_back(effval);
        rstate.stiffness.push_back(stiffval);
        rstate.damping.push_back(dampval);
    }
    
    if(sharedData->getNumClient().load() <= 0) {
        buffer->clear();
        buffer->add(rstate); 
    }
    else{
        buffer->add(rstate);   
    }
    
}

void CommunicationInterfaceWebServer::receiveReference()
{
    std::vector<double> vec;
    bool resp = sharedData->external_command->remove(vec);    
      
    int numjoint = _robot->getJointNum();
    if (resp){ 
      Eigen::VectorXd eigVec;
      eigVec.resize(numjoint);
      for (int i=0; i< vec.size(); i++){
         eigVec(i) = vec[i];
      }     
      _robot->setPositionReference(eigVec);
      
      //NOTE send velocityref for each joint
    
    }
    
      //set single joint value      
      JointIdMap pmap, vmap, emap , smap, dmap;
      _robot->getPositionReference(pmap);
      _robot->getVelocityReference(vmap);
      _robot->getEffortReference(emap);
      _robot->getStiffness(smap);
      _robot->getDamping(dmap);
      WebRobotStateRX rstate = sharedData->getRobotState();
      
      int i = 0;
      for ( auto id : rstate.joint_id){        
        pmap.at(id)= rstate.position_ref[i];
        vmap.at(id)= rstate.vel_ref[i];
        emap.at(id)= rstate.effort_ref[i];
        smap.at(id)= rstate.stiffness[i];
        dmap.at(id)= rstate.damping[i];
        i++;        
      }
      
      _robot->setPositionReference(pmap);
      _robot->setVelocityReference(vmap);
      _robot->setEffortReference(emap);
      _robot->setStiffness(smap);
      _robot->setDamping(dmap);     
     
}

void CommunicationInterfaceWebServer::resetReference()
{
      JointIdMap pmap, vmap, emap , smap, dmap;
      _robot->getPositionReference(pmap);
      _robot->getVelocityReference(vmap);
      _robot->getEffortReference(emap);
      _robot->getStiffness(smap);
      _robot->getDamping(dmap);
      WebRobotStateRX rstate;

      int i = 0;
      for ( auto id : rstate.joint_id){        
        rstate.position_ref[i] = pmap.at(id);
        rstate.vel_ref[i] = vmap.at(id);
        rstate.effort_ref[i] = emap.at(id);
        rstate.stiffness[i] = smap.at(id);
        rstate.damping[i] = dmap.at(id);
        i++;        
      }
      
      sharedData->setRobotState(rstate);  
}

bool CommunicationInterfaceWebServer::advertiseSwitch(const std::string& port_name)
{
    http_handler = std::make_shared<HttpHandler>(sharedData, buffer);
    http_civet_handler = std::make_shared<HttpCivetHandler>(http_handler);
    server->addHandler(SWITCH_URI, *http_civet_handler);
    server->addHandler(CMD_URI, *http_civet_handler);
    server->addHandler(STATUS_URI, *http_civet_handler);
    server->addHandler(MASTER_URI, *http_civet_handler);
    server->addHandler(ALLJOINT_URI, *http_civet_handler);
    server->addHandler(SINGLEJOINT_URI, *http_civet_handler);
    server->addHandler(PLUGIN_URI, *http_civet_handler);
    server->addHandler(STATE_URI, *http_civet_handler);
    server->addHandler(CHAINS_URI, *http_civet_handler);
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

std::string CommunicationInterfaceWebServer::getPluginStatus(const std::string& plugin_name)
{
    return sharedData->getStatus(plugin_name);
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

}