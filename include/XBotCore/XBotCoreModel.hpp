/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Murator (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_CORE_MODEL_HPP__
#define __X_BOT_CORE_MODEL_HPP__

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>

#include <fstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#define CHAIN_PER_GROUP 1

class XBotCoreModel : public srdf::Model {
private:
    
    std::string srdf_path;
    boost::shared_ptr<urdf::ModelInterface> urdf_model;
    KDL::Tree robot_tree;
    
    
    // vector for the chain names
    std::vector<std::string> chain_names;
    
    // vector for the disabled joints
    std::vector<std::string> disabled_joint_names;
    
    // map for the enabled joints in chains
    std::map<std::string, std::vector<std::string>> enabled_joints_in_chains;
    
    // map for the disabled joints in chains
    std::map<std::string, std::vector<std::string>> disabled_joints_in_chains;
    
    // TBD FT sensor and IMU

    
    
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
    
    
    bool parseSRDF() {

        std::vector<Group> actual_groups = getGroups();
        std::vector<DisabledJoint> actual_disabled_joints = getDisabledJoints();
        
        int group_num = actual_groups.size();
        
        // NOTE the last group is the "chains" group
        int chain_num = actual_groups[group_num - 1].subgroups_.size();
        chain_names.resize(chain_num);
        // fill the chain names vector
        for(int i = 0; i < chain_num; i++) {
            chain_names[i] = (actual_groups[group_num - 1].subgroups_[i]);
        }

        // put the disabled joint in the disabled_joint_names data structure
        int disabled_joint_num = actual_disabled_joints.size();
        disabled_joint_names.resize(disabled_joint_num);
        for( int i = 0; i < disabled_joint_num; i++) {  
           disabled_joint_names.push_back(actual_disabled_joints[i].name_); 
//             DPRINTF("Disable Joint -> name : %s\n", getDisabledJoints().at(i).name_.c_str());
        }
        
        // iterate over the groups, without the last one -> it is the "chains" group
        for(int i = 0; i < group_num - 1; i++) {
            // if the group represents a kinematic chain
            if(std::find(chain_names.begin(), chain_names.end(), actual_groups[i].name_) != chain_names.end() ) {
                // check the number of chain per group: it has to be 1
                if( actual_groups[i].chains_.size() != CHAIN_PER_GROUP)  {
                    // Error: only one chain per group
                    DPRINTF("ERROR: for the kinematic chain groups you can specify only one chain per group in your SRDF ( %s ): check %s group\n", srdf_path, actual_groups[i].name_.c_str() );
                    return false;
                }
                // fill the enabled/disabled joints in chain map
                if( !get_joints_in_chain( actual_groups[i].chains_[0].first, 
                                          actual_groups[i].chains_[0].second,
                                          enabled_joints_in_chains[actual_groups[i].name_],
                                          disabled_joints_in_chains[actual_groups[i].name_]) ) {
                    
                    DPRINTF("ERROR: get_joints_in_chain() failed\n");
                    return false;
                }

            }
            // not a kinematic chain : check for FT or IMU
            else {
                
            }

        }
        
//         // debug cout of the map
//         for (auto& kv : enabled_joints_in_chains) {
//             std::cout << kv.first << " has joints num : " << kv.second.size() << std::endl;
//             for(int i = 0; i < kv.second.size(); i++) {
//                 std::cout << kv.second.at(i) << std::endl;
//             }
//         }
//         
//          for (auto& kv : disabled_joints_in_chains) {
//             std::cout << kv.first << " has disabled joints num : " << kv.second.size() << std::endl;
//             for(int i = 0; i < kv.second.size(); i++) {
//                 std::cout << kv.second.at(i) << std::endl;
//             }
//         }
        
        return true;
    }
    
    bool get_joints_in_chain( std::string base_link, 
                              std::string tip_link,
                              std::vector<std::string>& enabled_joints_in_chain,
                              std::vector<std::string>& disabled_joints_in_chain) 
    {
        
        KDL::Chain actual_chain;
        if( robot_tree.getChain(base_link, tip_link, actual_chain) ) {
            int segments_num = actual_chain.getNrOfSegments();
            for( int i = 0; i < segments_num; i++) {
                KDL::Segment actual_segment = actual_chain.getSegment(i);
                KDL::Joint actual_joint = actual_segment.getJoint();
                
                // if the joint is revolute or prismatic
                if ( actual_joint.getTypeName() == "RotAxis" ||
                     actual_joint.getTypeName() == "TransAxis" ) { // TBD check this if needed
                    
                    // if the joint is enabled
                    if( !(std::find(disabled_joint_names.begin(), disabled_joint_names.end(), actual_joint.getName()) != disabled_joint_names.end() ) ) {
                        enabled_joints_in_chain.push_back(actual_joint.getName());
                    }
                    // disabled joint
                    else {
                        disabled_joints_in_chain.push_back(actual_joint.getName());
                    }
                }    
            }
            
            return true;
        }
        
        // chain not found
        return false;
    }
        
public:
    
    XBotCoreModel(void)
    {
    }
    
    boost::shared_ptr<urdf::ModelInterface> get_urdf_model(void) 
    {
        
        return urdf_model;
    }
    
    KDL::Tree get_robot_tree(void)
    {
        
        return robot_tree;
    }
    
    std::vector<std::string> get_chain_names(void) 
    {
        return chain_names;
    }
    
    bool init(const std::string& urdf_filename, const std::string& srdf_filename)
    {
        // SRDF path
        srdf_path = srdf_filename;
        
        // load URDF model from file
        urdf_model = loadURDF(urdf_filename);
        
        // create the robot KDL tree from the URDF model
        if( !kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree) ) {
            DPRINTF("Failed to construct kdl tree");
            return false;
        }
        
        // load SRDF model from file
        bool ret = this->initFile(*urdf_model, srdf_filename);
        
        // parse the SRDF file and fill the data structure
        return (ret && parseSRDF());
    }

    bool get_enabled_joints_in_chain( std::string chain_name, std::vector<std::string>& enabled_joints) 
    {
        // check if the chain exists
        if( enabled_joints_in_chains.count(chain_name) ) {
            enabled_joints = enabled_joints_in_chains.at(chain_name);
            return true;
        }
        
        // chain does not exists
        DPRINTF("ERROR: requested chain in get_enabled_joints_in_chain() does not exist.\n");
        return false;
    }
    
    bool get_disabled_joints_in_chain( std::string chain_name, std::vector<std::string>& disabled_joints) 
    {
        // check if the chain exists
        if( enabled_joints_in_chains.count(chain_name) ) {
            disabled_joints = enabled_joints_in_chains.at(chain_name);
            return true;
        }
        
        // chain does not exists
        DPRINTF("ERROR: requested chain in get_disabled_joints_in_chain() does not exist.\n");
        return false;
    }
    
    
    ~XBotCoreModel() 
    {
    }
    
};

#endif //__X_BOT_CORE_MODEL_HPP__
