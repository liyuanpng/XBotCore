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

namespace XBot {

/**
* @brief SharedMemory provides a means for RT plugins to communicate with
* each other in a fast way. SharedObjects are created by the get() method,
* and provide access to the underlying objects via pointer semantics,
* meaning that it provides access to the underlying shared object of type T
* by the operator* and operator->. As the only requirement, types to be shared
* over SharedMemory must be default-constuctible.
*/
class SharedMemory {

public:

    typedef std::shared_ptr<SharedMemory> Ptr;

    template <typename T>
    SharedObject<T> advertise(const std::string& object_name);

    template <typename T>
    SharedObject<T> get(const std::string& object_name);

protected:

private:

    std::unordered_map<std::string, boost::any> _map;

};


template <typename T>
SharedObject<T> SharedMemory::advertise(const std::string& object_name)
{
    SharedObject<T> sh_obj;

    if( _map.count(object_name) > 0 ){

        // If the required shared object exists, try to cast it to the required type T
        try{
            sh_obj = boost::any_cast<SharedObject<T>>(_map.at(object_name));
        }
        catch(...){
            // If cast fails, print an error
            std::cerr << "ERROR in " << __func__ << "! Object " << object_name << " is not of type " << typeid(T).name() << "!" << std::endl;
            return sh_obj;
        }

        // Cast was ok, return shared object
        return sh_obj;
    }

    // If the object does not exist, add it and return the shared object
    sh_obj.reset(new T);
    _map[object_name] = boost::any(sh_obj);
    return sh_obj;


}


/**
 * @brief This method returns a SharedObject<T> which is linked to the provided object_name.
 * The SharedObject has pointer semantics, meaning that it provides access to the underlying
 * shared object of type T by the operator* and operator->. If a shared object with the provided
 * name does not exist, it is created and constructed with its default constructor.
 *
 * @Requirement: T must be default-constructible
 *
 * @param object_name The name of the requested shared object.
 * @return The requested SharedObject
 */
template <typename T>
SharedObject<T> SharedMemory::get(const std::string& object_name)
{

    SharedObject<T> sh_obj;

    if( _map.count(object_name) == 0 ){
        // If required object does not exist in the map, we create and return the shared object
        sh_obj.reset(new T);
        _map[object_name] = boost::any(sh_obj);
	return sh_obj;
    }

    // If the required object exists, check if it can be cast to the required type T
    try{
        sh_obj = boost::any_cast<SharedObject<T>>(_map.at(object_name));
    }
    catch(...){
        // If not, error
        std::cerr << "ERROR in " << __func__ << "! Object " << object_name << " is not of type " << typeid(T).name() << "!" << std::endl;
        return sh_obj;
    }

    // Cast went alright, return the shared object
    return sh_obj;
}




}

#endif