/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_CORE_MODEL_HPP__
#define __X_BOT_CORE_MODEL_HPP__

#include <XBotCore/IXBotModel.h>

#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>

#include <fstream>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#define CHAIN_PER_GROUP 1

namespace XBot 
{
    class XBotCoreModel;
}

typedef std::map<int, std::string>  Rid2JointMap;
typedef std::map<std::string, int>  Joint2RidMap;

class XBot::XBotCoreModel : public srdf::Model,
                            public XBot::IXBotModel
{
private:
    
    std::string srdf_path;
    std::string joint_map_config_path;
    boost::shared_ptr<urdf::ModelInterface> urdf_model;
    KDL::Tree robot_tree;
    
        
    /**
     * @brief map between the chain name and the id of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<int>> robot;
    
    /**
     * @brief map between joint robot id and joint name
     * 
     */
    Rid2JointMap rid2joint;
    
    /**
     * @brief map between joint name and joint robot id
     * 
     */
    Joint2RidMap joint2rid;
        
    
    // vector for the chain names
    std::vector<std::string> chain_names;
    
    // vector for the disabled joints
    std::vector<std::string> disabled_joint_names;
    
    // map for the enabled joints in chains
    std::map<std::string, std::vector<std::string>> enabled_joints_in_chains;
    
    // map for the disabled joints in chains
    std::map<std::string, std::vector<std::string>> disabled_joints_in_chains;
    
    // TBD FT sensor and IMU

    
    
    /**
     * @brief load the URDF filename
     * 
     * @param filename the URDF filename
     * @return boost::shared_ptr< urdf::ModelInterface > the URDF model interface
     */
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
    
    
    /**
     * @brief parse the SRDF in order to have the information of the robot chains and enabled/disabled joints.
     * 
     * @return bool true on success, false otherwise: one possible cause can be the incorrect SRDF format.
     */
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
            // TBD not a kinematic chain : check for FT or IMU
            else {
                
            }

        }
        
        // TBD print #ifdef DEBUG and put the DPRINF instead of the cout
//         // debug cout of the map
//         for (auto& kv : enabled_joints_in_chains) {
//             std::cout << kv.first << " has joints num : " << kv.second.size() << std::endl;
//             for(int i = 0; i < kv.second.size(); i++) {
//                 std::cout << kv.second.at(i) << std::endl;
//             }
//         }
//          for (auto& kv : disabled_joints_in_chains) {
//             std::cout << kv.first << " has disabled joints num : " << kv.second.size() << std::endl;
//             for(int i = 0; i < kv.second.size(); i++) {
//                 std::cout << kv.second.at(i) << std::endl;
//             }
//         }
        
        return true;
    }
    
    /**
     * @brief get the enabled/disabled actuated (RotAxis or TransAxis) in the chain from base_link to tip_link
     * 
     * @param base_link base link of the chain
     * @param tip_link tip link of the chain
     * @param enabled_joints_in_chain vector that will be filled with the enabled joint names in the chain
     * @param disabled_joints_in_chain vector that will be filled with the disabled joint names in the chain
     * @return bool true on success, false if the chain does not exists in the robto tree
     */
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
                if ( actual_joint.getTypeName() == "RotAxis"   ||
                     actual_joint.getTypeName() == "TransAxis" ||  // TBD check this if needed
                     actual_joint.getName() == "l_handj" || actual_joint.getName() == "r_handj") {   // TBD check the model for the hands
                    
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
    
    /**
     * @brief getter for the enabled joints vector in the chain
     * 
     * @param chain_name the requested chain name
     * @param enabled_joints vector that will be filled with the enabled joint names
     * @return bool true if the chain exists, false otherwise
     */
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
    
    /**
     * @brief getter for the disabled joints vector in the chain
     * 
     * @param chain_name the requested chain name
     * @param disabled_joints vector that will be filled with the disabled joint names
     * @return bool true if the chain exists, false otherwise
     */
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
    
    
    void parseJointMap(void)
    {
        // read the joint map config file -> we choose to separate it from the one used by XBotCore and ec_boards_iface
        YAML::Node joint_map_cfg = YAML::LoadFile(joint_map_config_path);
        const YAML::Node& joint_map = joint_map_cfg["joint_map"];

        // iterate over the node
        for(YAML::const_iterator it=joint_map.begin();it != joint_map.end();++it) {
            int tmp_rid = it->first.as<int>();
            std::string tmp_joint = it->second.as<std::string>();
            // fill the maps 
            rid2joint[tmp_rid] = tmp_joint;
            joint2rid[tmp_joint] = tmp_rid;

    //         DPRINTF("rid2joint : rid -> %d ==> joint -> %s\n", tmp_rid, rid2joint[tmp_rid].c_str());
    //         DPRINTF("joint2rid : joint -> %s ==> rid -> %d\n", tmp_joint.c_str(), joint2rid[tmp_joint]);
        }
    }

public:
    
    XBotCoreModel(void)
    {
    }
    
    /**
     * @brief getter for the URDF ModelInterface
     * 
     * @return boost::shared_ptr< urdf::ModelInterface > the URDF ModelInterface
     */
    boost::shared_ptr<urdf::ModelInterface> get_urdf_model(void) 
    {
        
        return urdf_model;
    }
    
    /**
     * @brief getter for the KDL robot tree
     * 
     * @return KDL::Tree the KDL robot tree
     */
    KDL::Tree get_robot_tree(void)
    {
        
        return robot_tree;
    }
    
    /**
     * @brief getter for the chain names vector
     * 
     * @return std::vector< std::::string> the chain names vector
     */
    std::vector<std::string> get_chain_names(void) 
    {
        return chain_names;
    }
    
    /**
     * @brief initialization function for the model: it loads the URDF and parses the SRDF
     * 
     * @param urdf_filename URDF path
     * @param srdf_filename SRDF path
     * @param joint_map_config joint_map_config path
     * @return bool true on success, false otherwise
     */
    bool init(const std::string& urdf_filename, const std::string& srdf_filename, const std::string& joint_map_config)
    {
        // SRDF path
        srdf_path = srdf_filename;
        
        // joint_map_config path
        joint_map_config_path = joint_map_config;
        
        // load URDF model from file
        urdf_model = loadURDF(urdf_filename);
        
        // parse the Joint map
        parseJointMap();
        
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
    
    void generate_robot(void)
    {
        // generate the robot : map between tha chain name and the joint ids of the chain
        std::vector<std::string> actual_chain_names = get_chain_names();
        for( int i = 0; i < actual_chain_names.size(); i++) {
            std::vector<std::string> enabled_joints_name_aux;
            std::vector<int> enabled_joints_id_aux;
            if( get_enabled_joints_in_chain(actual_chain_names[i], enabled_joints_name_aux) ) {
                for( int j = 0; j < enabled_joints_name_aux.size(); j++ ) {
                    if( joint2rid.count(enabled_joints_name_aux[j]) ) {
                        enabled_joints_id_aux.push_back(joint2rid.at(enabled_joints_name_aux[j]));
                    }
                }
                robot[actual_chain_names[i]] = enabled_joints_id_aux;
            }
        }
    }
    
    virtual std::map<std::string, std::vector<int>> get_robot(void) final
    {
        return robot;
    }
          
    virtual std::string rid2Joint(int rId) final
    {
        return rid2joint.find(rId) != rid2joint.end() ? rid2joint[rId] : ""; 
    }
    
    virtual int joint2Rid(std::string joint_name) final
    {
        return joint2rid.find(joint_name) != joint2rid.end() ? joint2rid[joint_name] : 0;
    }
    
    
    ~XBotCoreModel() 
    {
    }
    
};

#endif //__X_BOT_CORE_MODEL_HPP__
