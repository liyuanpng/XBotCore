#ifndef _X_BOT_MOTION_CONTROL_
#define _X_BOT_MOTION_CONTROL_

#include <memory>
#include <map>

//  Yarp stuff
#include <stdint.h>
#include <vector>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>

//  XBot stuff
#include <XBotCore-interfaces/IXBotInit.h>



namespace XBot {
    namespace dev  {
    class XBotMotionControl;
    }
}

/**
 * @brief YARP Motion Control for XBOt : a DeviceDriver representing a kinematic chain of the robot.
 * 
 */
class XBot::dev::XBotMotionControl :    public DeviceDriver,
                                        public IAxisInfo,
//                                         public IPidControl,
//                                         public IControlCalibration2,
                                        public IEncodersTimed,
                                        public IMotorEncoders,
                                        public IPositionControl2,
                                        public IVelocityControl2,
                                        public IControlMode2,
                                        public IControlLimits2,
                                        public ITorqueControl,
                                        public IPositionDirect,
                                        public IInteractionMode,
                                        public IAmplifierControl,
                                        public IPWMControl,
                                        public IXBotInit

{
    
private:
    
    /**
     * @brief XBotInterface pointer
     * 
     */
    XBot::XBotInterface::Ptr _robot;
        
    /**
     * @brief Number of joints in the chain
     * 
     */
    int joints_num;     
    
    /**
     * @brief Chain name
     * 
     */
    std::string chain_name;     
    
    /**
     * @brief from YARP id to Joint id
     * 
     */
    std::map< int, int > Yid_to_Jid;
    
    /**
     * @brief from Joint id to YARP Id
     * 
     */
    std::map< int, int > Jid_to_Yid;
    
    /**
     * @brief control mode for each joint of the chain
     * 
     */
    std::vector<int> controlMode;
    
    /**
     * @brief interaction mode for each joint of the chain
     * 
     */
    std::vector<yarp::dev::InteractionModeEnum> interactionMode;
 
    /**
     * @brief min pos for each joint of the chain
     * 
     */
    std::vector<float> min_pos;
    
    /**
     * @brief max pos for each joint of the chain
     * 
     */
    std::vector<float> max_pos;
    
    /**
     * @brief last position reference XBot vector
     * 
     */
    Eigen::VectorXd xbot_last_ref_pos;
    
    /**
     * @brief joint map YARP bottle
     * 
     */
    yarp::os::Bottle joint_map;
    
//     XBot::JointIdMap _aux_id_map;

public:
    /**
     * @brief contructor
     * 
     */
    XBotMotionControl();
    
    /**
     * @brief destructor
     * 
     */
    ~XBotMotionControl();
    
    // XBot init
    virtual bool init(XBot::XBotInterface::Ptr robot);

    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    /////////   Axis info INTERFACE   /////////
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

//     /////////   PID INTERFACE   /////////
//     virtual bool setPid(int j, const Pid &pid);
//     virtual bool setPids(const Pid *pids);
//     virtual bool setReference(int j, double ref);
//     virtual bool setReferences(const double *refs);
//     virtual bool setErrorLimit(int j, double limit);
//     virtual bool setErrorLimits(const double *limits);
//     virtual bool getError(int j, double *err);
//     virtual bool getErrors(double *errs);
//     virtual bool getPid(int j, Pid *pid);
//     virtual bool getPids(Pid *pids);
//     virtual bool getReference(int j, double *ref);
//     virtual bool getReferences(double *refs);
//     virtual bool getErrorLimit(int j, double *limit);
//     virtual bool getErrorLimits(double *limits);
//     virtual bool resetPid(int j);
//     virtual bool disablePid(int j);
//     virtual bool enablePid(int j);
//     virtual bool setOffset(int j, double v);

    /////////// POSITION CONTROL INTERFACE 
    virtual bool setPositionMode();
    virtual bool positionMove(int j, double ref);
    virtual bool positionMove(const double *refs);
    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double *deltas);
    virtual bool checkMotionDone(bool *flag);
    virtual bool checkMotionDone(int j, bool *flag);
    virtual bool setRefSpeed(int j, double sp);
    virtual bool setRefSpeeds(const double *spds);
    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double *accs);
    virtual bool getRefSpeed(int j, double *ref);
    virtual bool getRefSpeeds(double *spds);
    virtual bool getRefAcceleration(int j, double *acc);
    virtual bool getRefAccelerations(double *accs);
    virtual bool stop(int j);
    virtual bool stop();
    // Position Control2 Interface
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);
    
    
    
    /////////// VELOCITY CONTROL INTERFACE 
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);
    virtual bool setVelPid(int j, const yarp::dev::Pid &pid);
    virtual bool setVelPids(const yarp::dev::Pid *pids);
    virtual bool getVelPid(int j, yarp::dev::Pid *pid);
    virtual bool getVelPids(yarp::dev::Pid *pids);
    virtual bool setVelocityMode();
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double *sp);

//     // calibration2raw
//     virtual bool setCalibrationParameters(int axis, const CalibrationParameters& params);
//     virtual bool calibrate2(int axis, unsigned int type, double p1, double p2, double p3);
//     virtual bool done(int j);

    // ControlMode
    virtual bool setPositionMode(int j);
    virtual bool setVelocityMode(int j);
    virtual bool setTorqueMode(int j);
    virtual bool setImpedancePositionMode(int j);
    virtual bool setImpedanceVelocityMode(int j);
    virtual bool setOpenLoopMode(int j);
    virtual bool getControlMode(int j, int *v);
    virtual bool getControlModes(int *v);

    // ControlMode 2
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);

    ////////////////////// EncoderInterface
    virtual bool getAxes(int *ax);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double *vals);
    virtual bool getEncoder(int j, double *v);
    virtual bool getEncoders(double *encs);
    virtual bool getEncoderSpeed(int j, double *sp);
    virtual bool getEncoderSpeeds(double *spds);
    virtual bool getEncoderAcceleration(int j, double *spds);
    virtual bool getEncoderAccelerations(double *accs);
    ///////////////////////// EncoderInterface
    virtual bool getEncodersTimed(double *encs, double *stamps);
    virtual bool getEncoderTimed(int j, double *encs, double *stamp);


    //////////////////////// BEGIN MotorEncoderInterface
    virtual bool getNumberOfMotorEncoders(int * num);
    virtual bool resetMotorEncoder(int m);
    virtual bool resetMotorEncoders();
    virtual bool setMotorEncoder(int m, const double val);
    virtual bool setMotorEncoders(const double *vals);
    virtual bool getMotorEncoder(int m, double *v);
    virtual bool getMotorEncoders(double *encs);
    virtual bool getMotorEncoderSpeed(int m, double *sp);
    virtual bool getMotorEncoderSpeeds(double *spds);
    virtual bool getMotorEncoderAcceleration(int m, double *spds);
    virtual bool getMotorEncoderAccelerations(double *accs);
    virtual bool getMotorEncodersTimed(double *encs, double *stamps);
    virtual bool getMotorEncoderTimed(int m, double *enc, double *stamp);
    virtual bool getMotorEncoderCountsPerRevolution(int m, double *v);
    virtual bool setMotorEncoderCountsPerRevolution(int m, const double cpr);
    ///////////////////////// END MotorEncoder Interface

    // Limits
    bool setLimits(int axis, double min, double max);
    bool getLimits(int axis, double *min, double *max);

    // Limits 2
    bool setVelLimits(int axis, double min, double max);
    bool getVelLimits(int axis, double *min, double *max);

    // Torque control
    bool setTorqueMode();
    bool getTorque(int j, double *t);
    bool getTorques(double *t);
    bool getBemfParam(int j, double *bemf);
    bool setBemfParam(int j, double bemf);
    bool getTorqueRange(int j, double *min, double *max);
    bool getTorqueRanges(double *min, double *max);
    bool setRefTorques(const double *t);
    bool setRefTorque(int j, double t);
    bool setRefTorques(const int n_joint, const int *joints, const double *t);
    bool getRefTorques(double *t);
    bool getRefTorque(int j, double *t);
    bool setTorquePid(int j, const Pid &pid);
    bool setTorquePids(const Pid *pids);
    bool setTorqueErrorLimit(int j, double limit);
    bool setTorqueErrorLimits(const double *limits);
    bool getTorqueError(int j, double *err);
    bool getTorqueErrors(double *errs);
    bool getTorquePidOutput(int j, double *out);
    bool getTorquePidOutputs(double *outs);
    bool getTorquePid(int j, Pid *pid);
    bool getTorquePids(Pid *pids);
    bool getTorqueErrorLimit(int j, double *limit);
    bool getTorqueErrorLimits(double *limits);
    bool resetTorquePid(int j);
    bool disableTorquePid(int j);
    bool enableTorquePid(int j);
    bool setTorqueOffset(int j, double v);
    bool getMotorTorqueParams(int j, MotorTorqueParameters *params);
    bool setMotorTorqueParams(int j, const MotorTorqueParameters params);

    // PositionDirect Interface
    bool setPositionDirectMode();
    bool setPosition(int j, double ref);
    bool setPositions(const int n_joint, const int *joints, double *refs);
    bool setPositions(const double *refs);
    bool getRefPosition(const int j, double *ref);
    bool getRefPositions(double *refs);
    bool getRefPositions(const int n_joint, const int *joints, double *refs);

    // InteractionMode interface
    bool getInteractionMode(int j, yarp::dev::InteractionModeEnum* _mode);
    bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    bool setInteractionMode(int j, yarp::dev::InteractionModeEnum _mode);
    bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);
    
    // AmplifierControl interface
    virtual bool disableAmp(int j);
    virtual bool enableAmp(int j);
    virtual bool getAmpStatus(int j, int* v);
    virtual bool getCurrent(int j, double* val);
    virtual bool getAmpStatus(int* st);
    virtual bool getCurrents(double* vals);
    virtual bool getMaxCurrent(int j, double* v);
    virtual bool setMaxCurrent(int j, double v);
    virtual bool getPWM(int j, double* val);
    
    // PWMControl interface
    virtual bool getDutyCycle(int m, double* val);
    virtual bool getDutyCycles(double* vals);
    virtual bool getNumberOfMotors(int* number);
    virtual bool getRefDutyCycle(int m, double* ref);
    virtual bool getRefDutyCycles(double* refs);
    virtual bool setRefDutyCycle(int m, double ref);
    virtual bool setRefDutyCycles(const double* refs);



};

#endif // _X_BOT_MOTION_CONTROL_