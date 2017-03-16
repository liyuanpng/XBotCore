#ifndef __XBOT_UTILS_H__
#define __XBOT_UTILS_H__

#include <stdint.h>

#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

#include <fstream>
#include <sstream>
#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>

namespace b_acc = boost::accumulators;

namespace XBot {
    
typedef b_acc::accumulator_set<uint64_t,
        b_acc::features<
             b_acc::tag::count
            ,b_acc::tag::mean
            ,b_acc::tag::min
            ,b_acc::tag::max
            ,b_acc::tag::variance(b_acc::lazy)
            ,b_acc::tag::error_of<b_acc::tag::mean>
        >
    > stat_t;

inline void print_stat(stat_t &s) {

    std::ostringstream oss;

    if (b_acc::count(s) > 0) {
        oss << "\tCount " << b_acc::count(s) << std::endl;
        oss << "\tMean " << (uint64_t)b_acc::mean(s);
        oss << "\tMin " << b_acc::min(s);
        oss << "\tMax " << b_acc::max(s);
        oss << "\tVar " << b_acc::variance(s);
        oss << "\tErrOfMean " << b_acc::error_of<b_acc::tag::mean>(s);
        oss << std::endl;
    } else {
        oss << "No data ..." << std::endl;
    }
    DPRINTF("%s", oss.str().c_str());
}


template <typename T>
inline void dump_buffer(std::string filename, T t) {

    char buffer[1024];
    std::ofstream log_file(filename.c_str());

    for ( typename T::iterator it=t.begin(); it!=t.end(); it++ ) {
        (*it).sprint(buffer, sizeof(buffer));
        log_file << std::string(buffer) <<  std::endl;
    }
    log_file << std::flush;
    log_file.close();

}


#define NSEC_PER_SEC    1000000000ULL

inline uint64_t get_time_ns(clockid_t clock_id=CLOCK_MONOTONIC)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(clock_id, &ts);
    time_ns = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
    return time_ns;
}


/* add ns to timespec */
inline void add_timespec(struct timespec *ts, int64_t addtime)
{
    int64_t sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if ( ts->tv_nsec > NSEC_PER_SEC ) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

}



#endif //__XBOT_UTILS_H__
