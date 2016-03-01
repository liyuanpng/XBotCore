#include <XBotCore/XBotCore.h>

XBotCore::XBotCore(const char* config_yaml) : Ec_Thread_Boards_base(config_yaml)
{

    // XBotCore config
    const YAML::Node& root_cfg =  get_config_YAML_Node();
    const YAML::Node& x_bot_core_config = root_cfg["x_bot_core"];
    thread_name = x_bot_core_config["name"].as<std::string>();
    // set thread name
    name = thread_name.c_str();
    
    // non periodic 
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

}

XBotCore::~XBotCore() {
    
}

void XBotCore::init_preOP(void) {

}

void XBotCore::parseSRDF() {

    int group_num = model.getGroups().size();
    for(int i = 0; i < group_num; i++) {
        DPRINTF("name : %s\n", model.getGroups().at(i).name_.c_str());
        DPRINTF("joints num : %d\n", model.getGroups().at(i).joints_.size());
        for(int j = 0; j < model.getGroups().at(i).joints_.size(); j++) {
            DPRINTF("%s\n", model.getGroups().at(i).joints_.at(j).c_str());
        }
        DPRINTF("-------------------------");
        DPRINTF("links num: %d\n", model.getGroups().at(i).links_.size());
        for(int j = 0; j < model.getGroups().at(i).links_.size(); j++) {
            DPRINTF("%s\n", model.getGroups().at(i).links_.at(j).c_str());
        }
        DPRINTF("-------------------------");
        DPRINTF("chains num: %d\n", model.getGroups().at(i).chains_.size());
        for(int j = 0; j < model.getGroups().at(i).chains_.size(); j++) {
            DPRINTF("%s -> %s\n", model.getGroups().at(i).chains_.at(j).first.c_str(),
                                  model.getGroups().at(i).chains_.at(j).second.c_str()
            );
        }
        DPRINTF("-------------------------");
        DPRINTF("subgroups num: %d\n", model.getGroups().at(i).subgroups_.size());
        for(int j = 0; j < model.getGroups().at(i).subgroups_.size(); j++) {
            DPRINTF("%s\n", model.getGroups().at(i).subgroups_.at(j).c_str());
        }

    }
}


void XBotCore::init_OP(void) {
    
    // TBD path not like this
    model.init("/home/embedded/src/XBotCore/configs/urdf/bigman.urdf",
                    "/home/embedded/src/XBotCore/configs/srdf/bigman_config.srdf" );

    parseSRDF();
}

int XBotCore::user_loop(void) {

//     DPRINTF("%d\n",get_rtt(1));
}

uint16_t XBotCore::get_rtt(int slave_id)
{
    return motors[slave_id]->getRxPDO().rtt;
}
