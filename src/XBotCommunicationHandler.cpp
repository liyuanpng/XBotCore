#include <XBotCore/XBotCommunicationHandler.h>

#include <sys/stat.h>
#include <fcntl.h>

XBot::XBotCommunicationHandler::XBotCommunicationHandler()
{

}

bool XBot::XBotCommunicationHandler::init(std::string config_file)
{
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
        return false;
    }

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();
    
    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
        return false;
    }
    
    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    
    // initialize all the fd
    for(auto& c : robot) {
        for(int i=0; i< c.second.size(); i++) {
            int actual_fd = open((std::string("/proc/xenomai/registry/rtipc/xddp/") + std::string("Motor_id_") + std::to_string(c.second[i])).c_str(), O_RDONLY);
            // TBD check actual_fd
            fd[c.second[i]] = actual_fd;
        }
    }
    
    iit::ecat::advr::McEscPdoTypes::pdo_rx pdo;
    for( auto& f: fd) {
        int n_bytes = read(f.second, (void*)&pdo, sizeof(pdo));
        DPRINTF("read : %d bytes\n", n_bytes);
        DPRINTF("read : %f bytes\n", pdo.link_pos);
        std::fflush(stdout);
    }

    
    
    return true;

}


XBot::XBotCommunicationHandler::~XBotCommunicationHandler()
{

}
