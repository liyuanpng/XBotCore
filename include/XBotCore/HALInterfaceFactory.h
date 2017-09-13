/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
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


#ifndef __HAL_INTERFACE_FACTORY_H__
#define __HAL_INTERFACE_FACTORY_H__

#include <map>
#include <string>
#include <iostream>

#include <dlfcn.h>

#include <XBotCore/HALInterface.h>

class HALInterfaceFactory {

public:
  
  static std::shared_ptr<HALInterface> getFactory(const std::string& file_name, const std::string& lib_name,  const char * config);

  static void unloadLib(const std::string& file_name);

private:

  HALInterfaceFactory() = delete;
  
  static std::map<std::string, void*> handles;
  
  static bool computeAbsolutePath ( const std::string& input_path,
                                      const std::string& middle_path,
                                      std::string& absolute_path );
  

};


#endif