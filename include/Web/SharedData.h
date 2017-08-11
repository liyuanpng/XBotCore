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

#ifndef __SHARED_DATA_WEBSERVER_H__
#define __SHARED_DATA_WEBSERVER_H__

#include <CircularBuffer.h>
#include <mutex>
#include <memory>
#include <atomic>
#include <map>
#include <vector>
#include <iostream>


class SharedData {
   
  public:
  
    SharedData();
     
    std::map<std::string, std::string> getAllStatus();
    
    std::map<int, double> getJointMap();
    
    void insertJoint(int key, double val);
    
    double getJoint(int key);
    
    void clearJointMap();
    
    void insertSwitch(std::string key, std::string val);
    
    std::string getSwitch(std::string key);

    void insertCmd(std::string key, std::string val);
    
    std::string getCmd(std::string key);

    void insertStatus(std::string key, std::string val);
    
    std::string getStatus(std::string key);
    
    void setMaster(std::string& val);
    
    void getMaster(std::string& val);
    
    void increaseNumClient();
    
    void decreaseNumClient();
    
    std::atomic<int>& getNumClient();
    
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
    
    std::map<int,double> _joint_map;
    std::mutex j_mutex;
   
};


#endif //__SHARED_DATA_WEBSERVER_H__