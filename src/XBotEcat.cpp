#include <XBotCore/XBotEcat.h>

#include <string.h>

XBotEcat::XBotEcat(const char* config_yaml) : Ec_Thread_Boards_base(config_yaml)
{
    // set thread name
    const YAML::Node& board_ctrl = root_cfg["x_bot_ecat"]; // TBD check that the Node is defined
    set_thread_name(board_ctrl["name"].as<std::string>()); // TBD check that name is defined
    
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,1};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
}

void XBotEcat::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBotEcat::get_thread_name(void)
{
    return thread_name;
}

void XBotEcat::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBotEcat::set_thread_priority()
{

    // set scheduler policy
#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}


void XBotEcat::init_preOP(void) 
{
//     start_motors(CTRL_SET_POS_MODE);
    return;
}

void XBotEcat::init_OP(void)
{
    control_init();
    return;
}

int XBotEcat::user_loop(void) {
    
    // call the control loop
    return control_loop();
}


XBotEcat::~XBotEcat() {
    
}
