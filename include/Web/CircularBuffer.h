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

#ifndef __CIRCULAR_BUFFER_WEBSERVER_H__
#define __CIRCULAR_BUFFER_WEBSERVER_H__

#include <mutex>
#include <boost/circular_buffer.hpp>

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
#endif //__CIRCULAR_BUFFER_WEBSERVER_H__