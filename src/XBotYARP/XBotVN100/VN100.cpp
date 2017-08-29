#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include <VN100.hpp>
#include "vectornav.h"

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
    counter = 0;
    comPortName = "";
    com_baudrate = -1;
}

VN100::~VN100() { }

bool VN100::open(yarp::os::Searchable &config)
{
    bool ret = true;
    std::cout << "config params for VN-100:\n" << config.toString() << std::endl;

    if(config.check("imu_id") && config.find("imu_id").isString())
    {
        comPortName = config.find("imu_id").asString();
    }
    else
    {
        std::cout << "ERROR: imu_id param not found!!" << std::endl;
        ret = false;
    }
    comPortName = "/dev/imu_" + comPortName;

    if(config.check("baudrate")  && config.find("baudrate").isInt())
    {
        com_baudrate = config.find("baudrate").asInt();
    }
    else
    {
        std::cout << "ERROR: baudrate param not found!!" << std::endl;
        ret = false;
    }

    if(!ret)
        return false;

    std::cout << "VN-100 module using the following parameters:" << std::endl;
    std::cout << "  imu_id " << comPortName << std::endl;
    std::cout << "  baudrate   " << com_baudrate << std::endl;

    errorCode = vn100_connect( &vn100, comPortName.c_str(), com_baudrate);

    /* Make sure the user has permission to use the COM port. */
    if (errorCode == VNERR_PERMISSION_DENIED) {

        printf("Current user does not have permission to open the COM port.\n");
        printf("Add your user to the dialout group with the command: \n");
        printf("sudo adduser 'username' dialout \n");
        printf("Or try running again using 'sudo'.\n");
        return false;
    }
    else if (errorCode != VNERR_NO_ERROR)
    {
        printf("Error encountered when trying to connect to the sensor.\n");
        return false;
    }


    //     /* Disable ASCII asynchronous messages since we want to demonstrate the
    //      *       the binary asynchronous messages. */
    errorCode = vn100_setAsynchronousDataOutputType( &vn100, VNASYNC_OFF, true);

    if (errorCode != VNERR_NO_ERROR)
    {
        printf("Error encountered when setting vn100_setAsynchronousDataOutputType.\n");
        return false;
    }

    /* Now configure the binary messages output. Notice how the configuration
     *       flags can be joined using the binary OR. */
    errorCode = vn100_setBinaryOutput1Configuration(
        &vn100,
        BINARY_ASYNC_MODE_SERIAL_1,     /* Data will be output on serial port 1. This should be the one we are connected to now. */
        4,                            /* Outputting binary data at 4 Hz (800 Hz on-board filter / 200 = 4 Hz). */
        BG1_YPR | BG1_MAG_PRES /*| BG1_ANGULAR_RATE */ | BG1_ACCEL,
        BG3_NONE,
        BG5_NONE,
        true);

    if (errorCode != VNERR_NO_ERROR)
    {
        printf("Error encountered when setting vn100_setBinaryOutput1Configuration.\n");
        return false;
    }

//     this->start();
    return true;
}


bool VN100::read(yarp::sig::Vector &out)
{ 
    /* The library is handling and storing asynchronous data by itself.
     C alling this *fun*ction retrieves the most recently processed
     asynchronous data packet. */

    vn100_getCurrentAsyncData(&vn100, &imu_data);

    out[0] = (double) imu_data.ypr.yaw;
    out[1] = (double) imu_data.ypr.pitch;
    out[2] = (double) imu_data.ypr.roll;
    out[3] = (double) imu_data.acceleration.c0;
    out[4] = (double) imu_data.acceleration.c1;
    out[5] = (double) imu_data.acceleration.c2;
    out[6] = (double) imu_data.velocity.c0;
    out[7] = (double) imu_data.velocity.c1;
    out[8] = (double) imu_data.velocity.c2;
    out[9] = (double) imu_data.magnetic.c0;
    out[10] = (double) imu_data.magnetic.c1;
    out[11] = (double) imu_data.magnetic.c2;
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
    errorCode = vn100_disconnect(&vn100);

    if (errorCode != VNERR_NO_ERROR)
    {
        printf("Error encountered when trying to disconnect from the sensor.\n");

        return 0;
    }

    return true;
}

yarp::os::Stamp VN100::getLastInputStamp()
{
    //     return yarp::os::Stamp(counter, (double) imu_data.timeStamp);
    return yarp::os::Stamp(counter, (double) yarp::os::Time::now());   // TBD use timestamp from imu if available
}



