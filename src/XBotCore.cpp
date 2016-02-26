#include <XBotCore/XBotCore.h>

#define MID_POS(m,M)    (m+(M-m)/2)

XBotCore::XBotCore(const char* config_yaml) : Ec_Thread_Boards_base(config_yaml)
{

    name = "XBotCore";
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

void XBotCore::init_OP(void) {
    
    // TBD path not like this
    srdf_model.init("/home/embedded/src/XBotCore/configs/urdf/bigman.urdf",
                    "/home/embedded/src/XBotCore/configs/srdf/bigman_config.srdf" );
    DPRINTF("Groups : %d/n", srdf_model.getGroups().size());

}

int XBotCore::user_loop(void) {

//     DPRINTF("%d\n",get_rtt(1));
}

uint16_t XBotCore::get_rtt(int slave_id)
{
    return motors[slave_id]->getRxPDO().rtt;
}
