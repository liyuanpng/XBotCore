/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <XCM/IOPlugin.h>

namespace XBot {

bool IOPluginLoader::load(std::string plugin_name)
{
    _ioplugin_factory.reset(new shlibpp::SharedLibraryClassFactory<IOPlugin>);
    _ioplugin_class.reset(new shlibpp::SharedLibraryClass<IOPlugin>);

    std::string path_to_so = "lib" + plugin_name + ".so";
    computeAbsolutePath(path_to_so, LIB_MIDDLE_PATH, path_to_so);

    std::string factory_name = plugin_name + "_factory";
    _ioplugin_factory->open(path_to_so.c_str(), factory_name.c_str());

    if (!_ioplugin_factory->isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(_ioplugin_factory->getStatus()).c_str(),
               _ioplugin_factory->getLastNativeError().c_str());
        _load_success = false;
        return false;
    }

    _load_success = true;

    // open io plugin
    _ioplugin_class->open(*_ioplugin_factory);

    return true;


}

IOPlugin* IOPluginLoader::getPtr()
{
    if(!_load_success) return nullptr;
    else return &(*_ioplugin_class).getContent();
}

bool IOPluginLoader::computeAbsolutePath (const std::string& input_path,
                                                      const std::string& middle_path,
                                                      std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}

}