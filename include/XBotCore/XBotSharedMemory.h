/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
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

#ifndef __XBOTCORE_SHARED_MEMORY_H__
#define __XBOTCORE_SHARED_MEMORY_H__

#include <boost/any.hpp>
#include <unordered_map>
#include <memory>
#include <iostream>

namespace XBot {
 
    class SharedMemory {
      
    public:
        
        typedef std::shared_ptr<SharedMemory> Ptr;
        
        template <typename T>
        bool advertise(std::string object_name, std::shared_ptr<T> ptr);
        
        template <typename T>
        std::shared_ptr<T> get(std::string object_name);
        
        
    protected:
        
    private:
        
        std::unordered_map<std::string, boost::any> _map;
        
    };
    
template <typename T>
bool SharedMemory::advertise(std::string object_name, std::shared_ptr< T > ptr)
{
    if( _map.count(object_name) > 0 ){
        std::cerr << "ERROR in " << __func__ << "! Object " << object_name << " already defined!" << std::endl;
        return false;
    }
    
    _map[object_name] = boost::any(ptr);
    
    return true;
    
}

template <typename T>
std::shared_ptr< T > SharedMemory::get(std::string object_name)
{
    std::shared_ptr<T> ptr;
    
    if( _map.count(object_name) == 0 ){
        std::cerr << "ERROR in " << __func__ << "! Object " << object_name << " is not defined!" << std::endl;
        return ptr;
    }
    

    try{
        ptr = boost::any_cast<std::shared_ptr<T>>(_map.at(object_name));
    }
    catch(...){
        std::cerr << "ERROR in " << __func__ << "! Object " << object_name << " is not of type " << typeid(T) << "!" << std::endl;
        return ptr;
    }
    
    return ptr;
}

    
}

#endif