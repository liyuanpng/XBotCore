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

#ifndef __XBOTCORE_SHARED_OBJECT_H__
#define __XBOTCORE_SHARED_OBJECT_H__

#include <boost/any.hpp>
#include <unordered_map>
#include <memory>
#include <iostream>

#include <XCM/XBotThread.h>

namespace XBot {
    
class SharedMemory;

/**
* @brief SharedObjects is a wrapper around an object, which is used to
* share it over a SharedMemory. SharedObjects provide synchronized access to the underlying object
* via get()/set() methods.
*
*/
template<typename T>
class SharedObject {

public:
    
    friend class SharedMemory;

    SharedObject(std::string name = "");
    
    void set(const T& obj);
    
    void get(T& obj) const;
    
    T get() const;
    
    bool try_set(const T& obj);
    
    bool try_get(T& obj) const;
    
    explicit operator bool() const;
    
    bool isValid() const;

    const std::string& getName() const;



protected:

private:

    Mutex::Ptr _mtx;

    std::shared_ptr<T>  _obj;

    std::string _name;

};



template< typename T >
SharedObject<T>::SharedObject(std::string name):
    _mtx(std::make_shared<Mutex>()),
    _obj(std::make_shared<T>()),
    _name(name)
{
    
}

template< typename T >
void SharedObject<T>::get(T& obj) const
{
    std::lock_guard<Mutex> guard(*_mtx);
    
    obj = *_obj;
}

template< typename T >
T SharedObject<T>::get() const
{
    std::lock_guard<Mutex> guard(*_mtx);
    
    return *_obj;
}

template< typename T >
void SharedObject<T>::set(const T& obj)
{
    std::lock_guard<Mutex> guard(*_mtx);
    
    *_obj = obj;
}

template< typename T >
bool SharedObject<T>::try_get(T& obj) const
{
    if(!_mtx->try_lock()){
        return false;
    }
    
    obj = *_obj;
    
    _mtx->unlock();
    
    return true;
}

template< typename T >
bool SharedObject<T>::try_set(const T& obj)
{
    if(!_mtx->try_lock()){
        return false;
    }
    
    *_obj = obj;
    
    _mtx->unlock();
    
    return true;
}

template< typename T >
const std::string& SharedObject<T>::getName() const
{
    return _name;
}

template< typename T >
SharedObject<T>::operator bool () const 
{
    return _obj && _mtx && (_name != "");
}




}

#endif