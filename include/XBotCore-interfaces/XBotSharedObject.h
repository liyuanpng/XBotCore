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

namespace XBot {

/**
* @brief SharedObjects is a wrapper around a double pointer, which is used to
* share objects over a SharedMemory. SharedObjects provide access to the underlying
* via pointer semantics, meaning that it provides access to the underlying shared object of type T
* by the operator* and operator->.
*
*/
template<typename T>
class SharedObject {

public:



    SharedObject();

    SharedObject(T* dynamically_allocated_object);

    /**
     * @brief Resets the SharedObject with a dynamically-allocated one, provided as an argument.
     * SharedObject class must take the ownership of the provided pointer, meaning that the caller
     * must NOT call delete on it. The recomended way of calling reset is
     *
     * reset( new T(args...) );
     *
     * @param dynamically_allocated_object A pointer to an object of type T, obtained via a call to new.
     * @return void
     */
    void reset(T* dynamically_allocated_object);

    /**
     * @brief Pointer-like operator*, which gives R/W access to the underlying shared object.
     *
     * @return A reference to the underlying object.
     */
    T& operator*();

    /**
     * @brief Pointer-like operator*, which gives read access to the underlying shared object.
     *
     * @return A const reference to the underlying object.
     */
    const T& operator*() const;

    /**
     * @brief Pointer-link operator->(), used to access underlying object methods.
     *
     */
    T* operator->();

    /**
     * @brief Pointer-link operator->(), used to access underlying object methods.
     *
     */
    const T* operator->() const;

    /**
     * @brief Gets a shared pointer to the underlying object.
     */
    std::shared_ptr<T> get();

    /**
     * @brief Gets a shared pointer to the underlying object.
     */
    std::shared_ptr<const T> get() const;

    /**
     * @brief Set the SharedObject in a valid state, meaning that meaningful
     * information is present in it. The validity of a SharedObject can be
     * checked by calling isValid()
     *
     */
    bool setValid();

    /**
     * @brief Check if SharedObject was set in a valid state.
     */
    bool isValid() const;

    /**
     * @brief Check if the SharedObject is null.
     */
    bool isNull() const;

protected:

private:

    std::shared_ptr<std::shared_ptr<T>> _ptrptr;
    bool _is_valid;

};


template<typename T>
SharedObject<T>::SharedObject():
    _is_valid(false)
{
    _ptrptr = std::make_shared<std::shared_ptr<T>>();
}

template<typename T>
SharedObject<T>::SharedObject(T* dynamically_allocated_object):
    _is_valid(false)
{
    _ptrptr = std::make_shared<std::shared_ptr<T>>();
    *_ptrptr = std::shared_ptr<T>(dynamically_allocated_object);
}

template<typename T>
void SharedObject<T>::reset(T* dynamically_allocated_object)
{
    *_ptrptr = std::shared_ptr<T>(dynamically_allocated_object);
}

template<typename T>
std::shared_ptr<const T> SharedObject<T>::get() const
{
    return *_ptrptr;
}

template<typename T>
std::shared_ptr<T> SharedObject<T>::get()
{
    return *_ptrptr;
}

template<typename T>
const T& SharedObject<T>::operator*() const
{
    if(!isNull()) {
        return **_ptrptr;
    }
    else {
        std::cerr << "ERROR in XBotSharedObject: the requested shared objected was not initialized. You must call the reset() on the XBotSharedObject." << std::endl;
    }
}

template<typename T>
T& SharedObject<T>::operator*()
{
    if(!isNull()) {
        return **_ptrptr;
    }
    else {
        std::cerr << "ERROR in XBotSharedObject: the requested shared objected was not initialized. You must call the reset() on the XBotSharedObject." << std::endl;
    }
}

template<typename T>
const T* SharedObject<T>::operator->() const
{
    if(!isNull()) {
        return _ptrptr->get();
    }
    else {
        std::cerr << "ERROR in XBotSharedObject: the requested shared objected was not initialized. You must call the reset() on the XBotSharedObject." << std::endl;
    }
}

template<typename T>
T* SharedObject<T>::operator->()
{
    if(!isNull()) {
        return _ptrptr->get();
    }
    else {
        std::cerr << "ERROR in XBotSharedObject: the requested shared objected was not initialized. You must call the reset() on the XBotSharedObject." << std::endl;
    }
}

template<typename T>
bool SharedObject<T>::isNull() const
{
    return !*_ptrptr;
}

template<typename T>
bool SharedObject<T>::isValid() const
{
    return !isNull() && _is_valid;
}

template<typename T>
bool SharedObject<T>::setValid()
{
    if(*_ptrptr){
        _is_valid = true;
    }
    else{
        std::cerr << "ERROR in " << __func__ << "! Current shared object is null!" << std::endl;
        return false;
    }
}


}

#endif