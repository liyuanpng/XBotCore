/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Murator (2016-, luca.muratore@iit.it)
*/

#ifndef __XBOTCORE_SRDFDOM_H__
#define __XBOTCORE_SRDFDOM_H__

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

class XBotCore_srdfdom : public srdf::Model {
private:
    boost::shared_ptr<urdf::ModelInterface> loadURDF(const std::string& filename)
    {
        // get the entire file
        std::string xml_string;
        std::fstream xml_file(filename.c_str(), std::fstream::in);
        if (xml_file.is_open())
        {
            while (xml_file.good())
            {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
            }
            xml_file.close();
            return urdf::parseURDF(xml_string);
        }
        else
        {
            throw std::runtime_error("Could not open file " + filename + " for parsing.");
            return boost::shared_ptr<urdf::ModelInterface>();
        }
    }
        
public:
    
    XBotCore_srdfdom()
    {
    }
    
    bool init(const std::string& urdf_filename, const std::string& srdf_filename)
    {
        boost::shared_ptr<urdf::ModelInterface> u = loadURDF(urdf_filename);
        return this->initFile(*u, srdf_filename);
    }
    
    ~XBotCore_srdfdom() 
    {
    }
    
};

#endif //__XBOTCORE_SRDFDOM_H__
