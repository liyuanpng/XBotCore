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

#include <XBotCore-interfaces/XBotSharedObject.h>
#include <memory.h>

namespace XBot {

/**
* @brief SharedMemory provides a means for RT plugins to communicate with
* each other in a fast way. SharedObjects are created by the getSharedObject() method,
* and provide access to the underlying objects via get/set semantics, in a
* synchronized (thread-safe) way. As the only requirement, types to be shared
* over SharedMemory must be default-constuctible AND copy-assignable.
*/
class SharedMemory {

public:

    typedef std::shared_ptr<SharedMemory> Ptr;

    
    /**
    * @brief This method returns a SharedObject<T> which is linked to the provided object_name.
    * The SharedObject provide access to the underlying objects via get/set semantics, in a
    * synchronized (thread-safe) way. If a shared object with the provided
    * name does not exist, it is created and constructed with its default constructor.
    *
    * @Requirement: T must be default-constructible and copy-assignable.
    *
    * @param object_name The name of the requested shared object.
    * @return The requested SharedObject
    */
    template <typename T>
    SharedObject<T> getSharedObject(const std::string& object_name);
    
    /**
     * @brief Check if the requested object exists inside the shared memory
     * 
     * @param object_name The name of the requested shared object.
     * @return True if object exists.
     */
    bool hasObject(const std::string& object_name) const;
    
    /**
     * @brief Check if the requested object exists and has the provided type.
     * 
     * @param object_name The name of the requested shared object.
     * @return True if object exists and has the provided type.
     */
    template <typename T>
    bool objectIsType(const std::string& object_name) const;

protected:

private:

    std::map<std::string, boost::any> _obj_map;

};


template <typename T>
inline SharedObject<T> SharedMemory::getSharedObject(const std::string& object_name)
{

    if( _obj_map.count(object_name) == 0 ){
        
        _obj_map[object_name] = boost::any(SharedObject<T>(object_name));
        
    }
    
        
    SharedObject<T> * shobjptr = boost::any_cast< SharedObject<T> >(&(_obj_map.at(object_name)));
    
    if(!shobjptr){
        throw std::runtime_error("Could not create shared object! Provided type does not match!");
    }
    
    return *shobjptr;


}


inline bool SharedMemory::hasObject(const std::string& object_name) const
{
    return _obj_map.find(object_name) != _obj_map.end();
}


template <typename T>
inline bool SharedMemory::objectIsType(const std::string& object_name) const
{
    SharedObject<T> * shobjptr = boost::any_cast< SharedObject<T> >(&(_obj_map.at(object_name)));
    return shobjptr;
}





}

#endif