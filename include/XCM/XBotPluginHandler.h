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


#ifndef __XCM_XBOT_PLUGIN_HANDLER_H__
#define __XCM_XBOT_PLUGIN_HANDLER_H__

#include <XBotCore-interfaces/All.h>

#include <atomic>

#include <XCM/XBotControlPlugin.h>
#include <XCM/TimeProvider.h>
#include <XCM/XBotESCUtils.h>

#include <SharedLibraryClassFactory.h>
#include <SharedLibraryClass.h>
#include <XCM/XBotPluginStatus.h>

#include <XBotCore-interfaces/XBotHandle.h>
#include <XBotCore-interfaces/XBotOptions.h>
#include <XBotCore/HALInterface.h>

namespace XBot {

    class PluginHandler : public Handle
    {

    public:

        typedef std::shared_ptr<PluginHandler> Ptr;

        PluginHandler( RobotInterface::Ptr robot, 
                       TimeProvider::Ptr time_provider,
                       XBot::SharedMemory::Ptr shared_memory,
                       Options options);
        
        void update_plugins_set_name(const std::string& plugins_set_name);

        bool load_plugins();

        bool init_plugins(std::shared_ptr<HALInterface> halInterface   = nullptr,
                          std::shared_ptr< IXBotModel > model   = nullptr );

        void run();

        void close();
        
        std::shared_ptr<XBot::XBotControlPlugin> loadPlugin(const std::string& plugin_name);
        
        bool initPlugin(  std::shared_ptr<XBot::XBotControlPlugin> plugin_ptr,
                          const std::string& name);
        
        void unloadPlugin(const std::string& port_name);
        
        void replacePlugin(const std::string& name);

        std::vector<std::string>& getPluginsName();
        
        virtual const std::string& getPathToConfigFile() const;
        virtual RobotInterface::Ptr getRobotInterface() const;
        virtual SharedMemory::Ptr getSharedMemory() const;
        virtual RosUtils::RosHandle::Ptr getRosHandle() const;
        virtual bool getNrtPositionReference(XBot::JointIdMap& pos_id_map) const;
        virtual bool getNrtVelocityReference(XBot::JointIdMap& vel_id_map) const;
        virtual bool getNrtEffortReference(XBot::JointIdMap& eff_id_map) const;
        virtual bool getNrtImpedanceReference(XBot::JointIdMap& k_id_map, XBot::JointIdMap& d_id_map) const;
        
        ~PluginHandler();

    protected:

    private:
        
        void init_plugin_impl();
        void init_plugin_handle_stdexcept();
        void init_plugin_handle_except();

        bool init_xddp();
        
        void run_xddp();

        void fill_robot_state();

        void run_communication_handler();

        bool plugin_can_start(int plugin_idx);
        
        void fill_nrt_reference();
        
        void init_nrt_reference();

        static bool computeAbsolutePath ( const std::string& input_path,
                                          const std::string& midlle_path,
                                          std::string& absolute_path ); // TBD do it with UTILS
        
        const Options _options;

        XBot::TimeProvider::Ptr _time_provider;

        YAML::Node _root_cfg;

        bool _close_was_called;

        // Dynamic loading related variables
        std::vector<std::string> _rtplugin_names;
        std::vector<std::shared_ptr<XBot::XBotControlPlugin>> _rtplugin_vector;
        std::vector<bool> _plugin_init_success;
        std::vector<std::shared_ptr<XBot::Subscriber<XBot::Command>>> _plugin_switch;
        std::vector<std::shared_ptr<XBot::Publisher<XBot::Command>>>  _plugin_status;  
        
        std::vector<std::shared_ptr<XBot::Subscriber<XBot::Command>>> _plugin_cmd;
        std::vector<XBot::PublisherNRT<XBot::Command>> _command_pub_vector;
        
        std::vector<std::shared_ptr<PluginStatus>> _plugin_custom_status;
        std::vector<std::string> _plugin_state;
        std::vector<bool> _first_loop;

        std::vector<double> _last_time, _time, _period, _elapsed_time;

        std::map<int, XBot::PublisherRT<XBot::RobotState>> _motor_pub_map;
        std::map<int, XBot::RobotState> _robot_state_map;

        std::map<int, XBot::PublisherRT<XBot::RobotFT::pdo_rx>> _ft_pub_map;
        std::map<int, XBot::RobotFT::pdo_rx> _ft_state_map;

        std::map<int, XBot::PublisherRT<XBot::RobotIMU::pdo_rx>> _imu_pub_map;
        std::map<int, XBot::RobotIMU::pdo_rx> _imu_state_map;

        XBot::ESCUtils _esc_utils;
        
        std::shared_ptr<XBot::IXBotJoint> _xbot_joint;
        
        std::map< std::string, std::shared_ptr<XBot::XBotControlPlugin> > pluginMap;
        std::map < std::string , int > pluginPos;
        
        std::shared_ptr<HALInterface> _halInterface;

        int _communication_plugin_idx;
        int _logging_plugin_idx;
        int _nrtref_plugin_idx;

        RobotInterface::Ptr _robot;
        std::string _path_to_cfg;

        XBot::MatLogger::Ptr _pluginhandler_log;
        
        std::string _plugins_set_name;
        bool _is_RT_plugin_handler;
        
        XBot::SharedMemory::Ptr _shared_memory;
        std::shared_ptr< XBot::IXBotJoint> _joint;
        std::shared_ptr< XBot::IXBotFT > _ft;
        std::shared_ptr< XBot::IXBotIMU > _imu;
        std::shared_ptr< XBot::IXBotHand > _hand;
        std::shared_ptr< XBot::IXBotModel > _model;
        
        std::atomic<int> curr_plg;
        
        RosUtils::RosHandle::Ptr _roshandle;
        SharedObject<RosUtils::RosHandle::Ptr> _roshandle_shobj;
        
        JointIdMap _nrt_pos, _nrt_vel, _nrt_eff, _nrt_imp_k, _nrt_imp_d;
        std::map<std::string, XBot::SharedObject<XBot::JointIdMap>> _ref_map_so;

    };
}
#endif