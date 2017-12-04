#ifndef _X_BOT_FT_
#define _X_BOT_FT_

#include <memory>
#include <map>
#include <vector>

// YARP stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Semaphore.h>

// XBot stuff
#include <XBotCore-interfaces/IXBotInit.h>

namespace XBot {
    namespace dev  {
    class XBotFT;
    }
}

/**
 * @brief YARP F-T for XBot : a DeviceDriver representing a kinematic chain of the robot.
 * 
 */
class XBot::dev::XBotFT :    public yarp::dev::DeviceDriver,
                             public yarp::dev::IAnalogSensor,
                             public IXBotInit

{
    
private:

    /**
     * @brief Number of FT channel
     * 
     */
    int channels;     
    
    /**
     * @brief FT name
     * 
     */
    std::string ft_name;     
    
    /**
     * @brief XBotInterface pointer
     * 
     */
    XBot::XBotInterface::Ptr _robot;

    
public:
    /**
     * @brief contructor
     * 
     */
    XBotFT();
    
    /**
     * @brief destructor
     * 
     */
    ~XBotFT();
    
    virtual bool init(XBot::XBotInterface::Ptr robot);

    // Device Driver
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();
    
    //IAnalogSensor
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);
    virtual int calibrateChannel(int ch, double value);
    

};

#endif // _X_BOT_FT_