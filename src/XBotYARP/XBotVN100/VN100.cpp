#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include <VN100.hpp>

/* The serial device to use in the vn100_connect function is /dev/ttyUSB[n] with n is a progressive number starting from 0.
   For walkman robot, the udev rule will automatically create a symbolic link called imu_XXXXX pointing to the
   correct device file where XXXXX is the unique serial number for the IMU. To find out which IMU are mounted on
   the robot just do a 'ls /dev' and look for imu_XXXXX.
   In the configuration file you just need to enter the serial identifier (the XXXXX parT)

   baudrate to use by default is = 115200;
*/
using namespace yarp::dev;

inline uint64_t get_time_ns(clockid_t clock_id=CLOCK_MONOTONIC)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(clock_id, &ts);
    time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    return time_ns;
}

VN100::VN100()
{

}

VN100::~VN100() { }

bool VN100::open(yarp::os::Searchable &config)
{

    return true;
}

bool VN100::init(XBot::XBotInterface::Ptr robot)
{
    _robot = robot;
    return true;
}

bool VN100::read(yarp::sig::Vector &out)
{ 
//     out[0] = (double) imu_data.ypr.yaw;
//     out[1] = (double) imu_data.ypr.pitch;
//     out[2] = (double) imu_data.ypr.roll;
//     out[3] = (double) imu_data.acceleration.c0;
//     out[4] = (double) imu_data.acceleration.c1;
//     out[5] = (double) imu_data.acceleration.c2;
//     out[6] = (double) imu_data.velocity.c0;
//     out[7] = (double) imu_data.velocity.c1;
//     out[8] = (double) imu_data.velocity.c2;
//     out[9] = (double) imu_data.magnetic.c0;
//     out[10] = (double) imu_data.magnetic.c1;
//     out[11] = (double) imu_data.magnetic.c2;
    memset(&out, 0, sizeof(double) * 12);
    // update the counter
    counter++;
    return true;
}
bool VN100::getChannels(int *nc)
{
    *nc = 12;
    return true;
}
bool VN100::calibrate(int ch, double v) { return false; }

bool VN100::close()
{
    return true;
}

yarp::os::Stamp VN100::getLastInputStamp()
{
    //     return yarp::os::Stamp(counter, (double) imu_data.timeStamp);
    return yarp::os::Stamp(counter, (double) yarp::os::Time::now());   // TBD use timestamp from imu if available
}



