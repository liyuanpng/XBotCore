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

#include <SharedData.h>

SharedData::SharedData():master("") { 
    num_client.store(0);
    external_command = std::make_shared<Buffer<std::vector<double>>>(5);
}    
  
std::map<std::string, std::string> SharedData::getAllStatus(){
    std::lock_guard<std::mutex> locker(st_mutex); 
    return _status;
}

std::map<int, double> SharedData::getJointMap(){
    std::lock_guard<std::mutex> locker(j_mutex); 
    return _joint_map;
}

void SharedData::insertJoint(int key, double val){
    std::lock_guard<std::mutex> locker(j_mutex); 
    _joint_map[key] = val;
}

void SharedData::setRobotState(WebRobotStateRX rstate){
  
    std::lock_guard<std::mutex> locker(rstate_mutex); 
    _rstate = rstate;
}

WebRobotStateRX SharedData::getRobotState(){
  
    std::lock_guard<std::mutex> locker(rstate_mutex); 
    return _rstate;
}

double SharedData::getJoint(int key){
    std::lock_guard<std::mutex> locker(j_mutex); 
    return _joint_map[key];
}

void SharedData::clearJointMap(){
  std::lock_guard<std::mutex> locker(j_mutex); 
  _joint_map.clear();      
}

void SharedData::insertSwitch(std::string key, std::string val){
    std::lock_guard<std::mutex> locker(s_mutex); 
    _switch[key] = val;
}

std::string SharedData::getSwitch(std::string key){
    std::lock_guard<std::mutex> locker(s_mutex); 
    return _switch[key];
}

void SharedData::insertChain(std::string key, std::vector< std::vector<std::string> > val){
    std::lock_guard<std::mutex> locker(chain_mutex); 
    _chain[key] = val;
}

std::vector< std::vector<std::string> > SharedData::getChain(std::string key){
    std::lock_guard<std::mutex> locker(chain_mutex); 
    return _chain[key];
}

std::map<std::string, std::vector<std::vector<std::string>> > SharedData::getChainMap(){
    std::lock_guard<std::mutex> locker(chain_mutex);
    return _chain;
}

void SharedData::insertCmd(std::string key, std::string val){
    std::lock_guard<std::mutex> locker(c_mutex); 
    _cmd[key] = val;
}

std::string SharedData::getCmd(std::string key){
    std::lock_guard<std::mutex> locker(c_mutex); 
    return _cmd[key];
}

void SharedData::insertStatus(std::string key, std::string val){
    std::lock_guard<std::mutex> locker(st_mutex); 
    _status[key] = val;
}

std::string SharedData::getStatus(std::string key){
    std::lock_guard<std::mutex> locker(st_mutex); 
    return _status[key];
}

void SharedData::setMaster(std::string& val) {       
        std::lock_guard<std::mutex> locker(m_master);  
        master = val;
        return;       
}

void SharedData::getMaster(std::string& val) {       
        std::lock_guard<std::mutex> locker(m_master);  
        val = master;
        return;       
}

void SharedData::increaseNumClient(){      
  num_client++;
  std::cout<<"Num_Client "<< num_client.load()<<std::endl;
}

void SharedData::decreaseNumClient(){     
  num_client--;
  std::cout<<"Num_Client "<< num_client.load()<<std::endl;      
}

std::atomic<int>& SharedData::getNumClient(){
  return num_client;
}