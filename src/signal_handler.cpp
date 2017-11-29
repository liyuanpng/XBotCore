#include <signal.h>
#include <pthread.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <cstring>
#include <unistd.h> // needed for sysconf(int name);    
#include <malloc.h>
#include <sys/time.h> // needed for getrusage
#include <sys/resource.h> // needed for getrusage

#ifdef __XENO__
#include <rtdk.h>
#endif

#define MEM_LOCKED (500*1024*1024) // 500MB

#include <XBotInterface/RtLog.hpp>
using XBot::Logger;


static void warn_upon_switch ( int sig __attribute__ ( ( unused ) ) ) {
    
    Logger::warning("Real-time thread switched to secondary mode! \n");
    
    // handle rt to nrt contex switch
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace ( bt,sizeof ( bt ) /sizeof ( bt[0] ) );
    
    if( Logger::GetVerbosityLevel() <= Logger::Severity::LOW ){
        // dump backtrace
        backtrace_symbols_fd ( bt,nentries,fileno ( stdout ) );
    }
}


static void set_signal_handler ( __sighandler_t sig_handler ) {
    signal ( SIGINT, sig_handler );
    signal ( SIGINT, sig_handler );
    signal ( SIGKILL, sig_handler );
#if defined( __XENO__ ) || defined( __COBALT__ )
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal ( SIGXCPU, warn_upon_switch );
#endif
}


static int lock_mem ( int byte_size ) {
    // Allocate some memory
    int ret, i, page_size;
    char* buffer;
    struct rusage usage;

    // Now lock all current and future pages from preventing of being paged
    if ( ( ret=mlockall ( MCL_CURRENT | MCL_FUTURE ) ) < 0 ) {
        return ret;
    }

    // Turn off malloc trimming.
    mallopt ( M_TRIM_THRESHOLD, -1 );
    // Turn off mmap usage.
    mallopt ( M_MMAP_MAX, 0 );
    page_size = sysconf ( _SC_PAGESIZE );
    buffer = ( char* ) malloc ( byte_size );

    getrusage ( RUSAGE_SELF, &usage );
//     printf ( "Major-pagefaults:%ld, Minor Pagefaults:%ld\n", usage.ru_majflt, usage.ru_minflt );

    // Touch page to prove there will be no page fault later
    for ( i=0; i < byte_size; i+=page_size ) {
        // Each write to this buffer will *not* generate a pagefault.
        // Even if nothing has been written to the newly allocated memory, the physical page
        //  is still provisioned to the process because mlockall() has been called with
        //  the MCL_FUTURE flag
        buffer[i] = 0;
        // print the number of major and minor pagefaults this application has triggered
        //getrusage(RUSAGE_SELF, &usage);
        //printf("Major-pagefaults:%d, Minor Pagefaults:%d\n", usage.ru_majflt, usage.ru_minflt);
    }

    getrusage ( RUSAGE_SELF, &usage );
//     printf ( "Major-pagefaults:%ld, Minor Pagefaults:%ld\n", usage.ru_majflt, usage.ru_minflt );

    free ( buffer );
    // buffer is now released. As glibc is configured such that it never gives back memory to
    // the kernel, the memory allocated above is locked for this process. All malloc() and new()
    // calls come from the memory pool reserved and locked above. Issuing free() and delete()
    // does NOT make this locking undone. So, with this locking mechanism we can build C++ applications
    // that will never run into a major/minor pagefault, even with swapping enabled.


    //<do your RT-thing>


    return 0;
}
///////////////////////////////////////////////////////////////////////////////


void set_main_sched_policy ( int priority ) {

#if defined( __XENO__ ) || defined( __COBALT__ )
    int policy = SCHED_FIFO;
#else
    int policy = SCHED_OTHER;
#endif
    struct sched_param  schedparam;
    schedparam.sched_priority = priority; //sched_get_priority_max(policy);
    pthread_setschedparam ( pthread_self(), policy, &schedparam );

    return;
}

void main_common ( __sighandler_t sig_handler ) {
    int ret;

    set_signal_handler ( sig_handler );

#if defined( __XENO__ ) || defined( __COBALT__ )

    /* Prevent any memory-swapping for this program */
    //ret = mlockall(MCL_CURRENT | MCL_FUTURE);
//     ret = lock_mem ( MEM_LOCKED );
//     if ( ret < 0 ) {
//         printf ( "mlockall failed (ret=%d) %s\n", ret, strerror ( ret ) );
//         exit ( 0 );
//     }
#endif

#ifdef __XENO__
    
    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
    rt_print_auto_init ( 1 );
    
#endif

    return;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
