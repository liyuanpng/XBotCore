////////////////////////////////////////////////////////////
// thread util
//
// First created:  summer 2009, by A.Margan
//
// Revisions:
//
////////////////////////////////////////////////////////////

#ifndef __XBOT_THREAD_H__
#define __XBOT_THREAD_H__

#include <pthread.h>
#include <linux/sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <bits/local_lim.h>
#include <exception>
#include <typeinfo>
#include <iostream>
#include <memory>

#include <XCM/XBotUtils.h>

namespace XBot{
typedef struct {
    struct timeval task_time;
    struct timeval period;
} task_period_t;


typedef struct {
    char                thread_name[8];
    unsigned long long  start_time_ns;
    unsigned long long  loop_time_ns;
    unsigned long long  elapsed_time_ns;
    unsigned long long  _prev;
    unsigned long        overruns;
} task_time_stat_t;

}

namespace XBot{
class Thread_hook;
}

namespace XBot{
typedef XBot::Thread_hook* Thread_hook_Ptr;
void * rt_periodic_thread ( Thread_hook_Ptr );
void * rt_non_periodic_thread ( Thread_hook_Ptr );
void * nrt_thread ( Thread_hook_Ptr );

inline void tsnorm ( struct timespec *ts ) {
    while ( ts->tv_nsec >= NSEC_PER_SEC ) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}
}

class XBot::Thread_hook {

public:
    
    typedef std::shared_ptr<Thread_hook> Ptr;

    virtual ~Thread_hook();

    void create ( int rt, int cpu_nr );
    void stop ( void );
    void join ( void );

    int is_non_periodic();

    virtual void th_init ( void * ) = 0;
    virtual void th_loop ( void * ) = 0;

    static void * nrt_th_helper ( void * );
    static void * rt_th_helper ( void * );

protected:

    int _run_loop;

    const char *    name;
    task_period_t   period;

    pthread_t       thread_id;
    // pthread attribute
    int             schedpolicy;
    int             priority;
    int             stacksize = 0;

    friend void * rt_periodic_thread ( Thread_hook_Ptr );
    friend void * rt_non_periodic_thread ( Thread_hook_Ptr );
    friend void * nrt_thread ( Thread_hook_Ptr );

};


inline XBot::Thread_hook::~Thread_hook() {

    std::cout << "~" << typeid ( this ).name() << std::endl;
}

inline int XBot::Thread_hook::is_non_periodic() {

    return ( period.period.tv_sec == 0 && period.period.tv_usec == 1 );
}

inline void * XBot::Thread_hook::nrt_th_helper ( void *kls ) {

//     try {
        return nrt_thread ( ( Thread_hook_Ptr ) kls );
//     } catch ( std::exception &e ) {
//         DPRINTF ( "In function %s catch ::%s::\n\tThread %s quit\n",
//                   __FUNCTION__, e.what(), ( ( Thread_hook_Ptr ) kls )->name );
//         return 0;
//     }

}

inline void * XBot::Thread_hook::rt_th_helper ( void *kls )  {

//     try {

        if ( ( ( Thread_hook_Ptr ) kls )->is_non_periodic() ) {
            return rt_non_periodic_thread ( ( Thread_hook_Ptr ) kls );
        }
        return rt_periodic_thread ( ( Thread_hook_Ptr ) kls );

//     } catch ( std::exception &e ) {
//         DPRINTF ( "In function %s catch ::%s::\n\tThread %s quit\n",
//                   __FUNCTION__, e.what(), ( ( Thread_hook_Ptr ) kls )->name );
//         return 0;
//     }
}


inline void XBot::Thread_hook::stop() {
    _run_loop = 0;
}

inline void XBot::Thread_hook::join() {
    /*pthread_cancel(thread_id);*/ pthread_join ( thread_id, 0 );
}

inline void XBot::Thread_hook::create ( int rt=true, int cpu_nr=0 ) {

    int ret;
    pthread_attr_t      attr;
    struct sched_param  schedparam;
    cpu_set_t           cpu_set;
    size_t              dflt_stacksize;
    _run_loop = 1;

    CPU_ZERO ( &cpu_set );
    CPU_SET ( cpu_nr,&cpu_set );

    pthread_attr_init ( &attr );
    pthread_attr_setinheritsched ( &attr, PTHREAD_EXPLICIT_SCHED );
    pthread_attr_setschedpolicy ( &attr, schedpolicy );
    schedparam.sched_priority = priority;
    pthread_attr_setschedparam ( &attr, &schedparam );

    pthread_attr_getstacksize ( &attr, &dflt_stacksize );
    DPRINTF ( "default stack size %ld\n", dflt_stacksize );
    if ( stacksize > 0 ) {
        pthread_attr_setstacksize ( &attr, stacksize );
    }
    pthread_attr_setdetachstate ( &attr, PTHREAD_CREATE_JOINABLE );
    pthread_attr_setaffinity_np ( &attr, sizeof ( cpu_set ), &cpu_set );

#ifdef __XENO__
    if ( rt ) {
        ret = pthread_create ( &thread_id, &attr, &rt_th_helper, this );
    } else {
        ret = pthread_create ( &thread_id, &attr, &nrt_th_helper, this );
    }
#else
    ret = pthread_create ( &thread_id, &attr, &nrt_th_helper, this );
#endif

    pthread_attr_destroy ( &attr );

    if ( ret ) {
        DPRINTF ( "%s %d %s", __FILE__, __LINE__, name );
        perror ( "pthread_create fail" );

        exit ( 1 );
    }

}

#endif //__XBOT_THREAD_H__

