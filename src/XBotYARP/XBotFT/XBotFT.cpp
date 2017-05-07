#include <XCM/XBotYARP/XBotFT.h>

#include <XCM/XBotUtils.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>

XBot::dev::XBotFT::XBotFT()
{

}

bool XBot::dev::XBotFT::init(XBot::XBotInterface::Ptr robot)
{
    _robot = robot;
    return true;
}

bool XBot::dev::XBotFT::open(yarp::os::Searchable& config)
{
    // check for ft name
    if( config.check("ft_name") ) {
        ft_name = config.find("ft_name").asString();
    }
    else {
        DPRINTF("ERROR : XBotFT::open - ft_name not found\n");
        std::fflush(stdout);
        return false;
    }
    // check for ft channels
    if( config.check("channels") ) {
        channels = config.find("channels").asInt();
    }
    else {
        DPRINTF("ERROR : XBotFT::open - channels not found\n");
        std::fflush(stdout);
        return false;
    }
    
    return true;
}



bool XBot::dev::XBotFT::close()
{
    yTrace();
    return true;
}


int XBot::dev::XBotFT::read(yarp::sig::Vector& out)
{
    Eigen::Vector6d aux_ft;
    _robot->getForceTorque().at(ft_name)->getWrench(aux_ft);
    out.resize(aux_ft.size());
    for(int i = 0; i < aux_ft.size(); i++) {
        out[i] = aux_ft[i];
    }
    return AS_OK;
}

int XBot::dev::XBotFT::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int XBot::dev::XBotFT::calibrateSensor()
{
    return AS_OK;
}

int XBot::dev::XBotFT::calibrateChannel(int ch, double value)
{
    return AS_OK;
}

int XBot::dev::XBotFT::calibrateChannel(int ch)
{
    return AS_OK;
}

int XBot::dev::XBotFT::getState(int ch)
{
    return AS_OK;
}

int XBot::dev::XBotFT::getChannels()
{
    return channels;
}

XBot::dev::XBotFT::~XBotFT()
{

}

