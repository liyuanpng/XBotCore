#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include <XCM/XBotUtils.h>

#include <yarp/os/LogStream.h>
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

    // check for imu name
    if( config.check("imu_name") ) {
        imu_name = config.find("imu_name").asString();
    }
    else {
        DPRINTF("ERROR : VN100::open - imu_name not found\n");
        std::fflush(stdout);
        return false;
    }
    
    return true;
}

bool VN100::init(XBot::XBotInterface::Ptr robot)
{
    _robot = robot;
    return true;
}

bool VN100::read(yarp::sig::Vector &out)
{ 
    // get IMU pointer
    XBot::ImuSensor::ConstPtr imu =  _robot->getImu().at(imu_name);
    
    Eigen::Quaterniond orientation;
    Eigen::Vector3d acc, vel, euler_rpy;
    
    imu->getOrientation(orientation);
        
    Eigen::AngleAxisd::RotationMatrixType aa_rot(orientation);
    euler_rpy = aa_rot.eulerAngles(0,1,2);
    
    imu->getLinearAcceleration(acc);
    
    imu->getAngularVelocity(vel);
    
    out.resize(12);
    out[0] = (double) euler_rpy(0);
    out[1] = (double) euler_rpy(1);
    out[2] = (double) euler_rpy(2);
    out[3] = (double) acc(0);
    out[4] = (double) acc(1);
    out[5] = (double) acc(2);
    out[6] = (double) vel(0);
    out[7] = (double) vel(1);
    out[8] = (double) vel(2);
    out[9] = (double) 0.0;
    out[10] = (double) 0.0;
    out[11] = (double) 0.0;

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



