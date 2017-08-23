#include <XCM/XBotYARP/XBotMotionControl.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>

#include <XCM/XBotUtils.h>

#undef DEG2RAD
#undef RAD2DEG

#define DEG2RAD(x) (x*M_PI/180.0)
#define RAD2DEG(x) (x*180.0/M_PI) 


XBot::dev::XBotMotionControl::XBotMotionControl()
{

}

bool XBot::dev::XBotMotionControl::init(XBot::XBotInterface::Ptr robot)
{
    _robot = robot;
    //resize data structure
    controlMode.resize(joints_num);
    interactionMode.resize(joints_num);
    
    _robot->getMotorPosition(xbot_last_ref_pos);
    
    uint16_t control_mode;
    // id maps 
    for(int i = 0; i < joints_num; i++) {
        Yid_to_Jid[i] = joint_map.get(i).asInt();
        Jid_to_Yid[joint_map.get(i).asInt()] = i;
        
        // HACK TBD NOTE do it properly
        controlMode[i] = VOCAB_CM_POSITION_DIRECT;
        interactionMode[i] = VOCAB_IM_STIFF;
        
//         // get the control status cmd
//         _robot->get_ctrl_status_cmd(joint_map.get(i).asInt(), control_mode);
//         if(control_mode == 0x003B || control_mode == 0x003C) {
//             controlMode[i] = VOCAB_CM_POSITION_DIRECT;
//             interactionMode[i] = VOCAB_IM_STIFF;
//         }
//         else {
//             controlMode[i] = VOCAB_CM_IDLE;
//             interactionMode[i] = VOCAB_IM_STIFF;
//         }
    }
    return true;
}

// called after the init
bool XBot::dev::XBotMotionControl::open(yarp::os::Searchable &config)
{

    // check for chain name
    if( config.check("chain_name") ) {
        chain_name = config.find("chain_name").asString();
    }
    else {
        DPRINTF("ERROR : XBotMotionControl::open - chain_name not found\n");
        std::fflush(stdout);
        return false;
    }
    
    // check for joint map
    if( config.check("joint_map") ) {
        std::string joint_map_string = config.find("joint_map").asString();
        joint_map.fromString(joint_map_string);
        // joints num
        joints_num = joint_map.size();
        
    }
    else {
        DPRINTF("ERROR : XBotMotionControl::open - joint_map not found\n");
        std::fflush(stdout);
        return false;
    }
    
    for(auto& j : Yid_to_Jid) {
        DPRINTF("yid : %d - jid : %d \n", j.first, j.second);
        std::fflush(stdout);
    }

    return true;
}

bool XBot::dev::XBotMotionControl::close()
{
    yTrace();
    return true;
}

////////////////////// EncoderInterface

bool XBot::dev::XBotMotionControl::getAxes(int *ax)
{
    *ax = joints_num;
    return true;
}

bool XBot::dev::XBotMotionControl::getEncoder(int j, double* v)
{
    
    *v =  RAD2DEG(_robot->getChainMap().at(chain_name)->getJointPosition(j));
    return true;
}

bool XBot::dev::XBotMotionControl::getEncoders(double* encs)
{
    bool ret = true;
    for( int i = 0; i < joints_num; i++ ) {
        ret = getEncoder(i, &encs[i]) && ret;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::getEncoderTimed(int j, double *enc, double *stamp)
{
    *stamp = yarp::os::Time::now();
    *enc =  RAD2DEG(_robot->getChainMap().at(chain_name)->getJointPosition(j));
    return true;
}

bool XBot::dev::XBotMotionControl::getEncodersTimed(double *encs, double *stamps)
{
    bool ret = true;
    double stamp = yarp::os::Time::now();
    for( int i = 0; i < joints_num; i++ ) {
        encs[i] = RAD2DEG(_robot->getChainMap().at(chain_name)->getJointPosition(i));
        stamps[i] = stamp;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::getEncoderSpeed(int j, double* sp)
{
    *sp = RAD2DEG(_robot->getChainMap().at(chain_name)->getJointVelocity(j));
    return true;
}

bool XBot::dev::XBotMotionControl::getEncoderSpeeds(double* spds)
{
    bool ret = true;
    for( int i = 0; i < joints_num; i++ ) {
        ret = getEncoderSpeed(i, &spds[i]) && ret;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::getEncoderAcceleration(int j, double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getEncoderAccelerations(double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::resetEncoder(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::resetEncoders()
{
    return false;
}

bool XBot::dev::XBotMotionControl::setEncoder(int j, double val)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setEncoders(const double* vals)
{
    return false;
}



////////////////////// PositionDirect Interface

bool XBot::dev::XBotMotionControl::setPosition(int j, double ref)
{
    _robot->getChainMap().at(chain_name)->setPositionReference(j, DEG2RAD(ref));
    
    // save to last ref buff
    xbot_last_ref_pos[j] = DEG2RAD(ref);
    return true;
}

bool XBot::dev::XBotMotionControl::setPositions(const int n_joint, const int* joints, double* refs)
{
    bool success = true;
    for( int i = 0; i < n_joint; i++ ) {
        _robot->getChainMap().at(chain_name)->setPositionReference(joints[i], DEG2RAD(refs[joints[i]]));
        // save to last ref buff
        xbot_last_ref_pos[joints[i]] = DEG2RAD(refs[i]);
    }
    return success;
}


bool XBot::dev::XBotMotionControl::setPositions(const double* refs)
{
    bool success = true;
    for( int i = 0; i < joints_num; i++ ) {
        _robot->getChainMap().at(chain_name)->setPositionReference(i, DEG2RAD(refs[i]));
        
        // save to last ref buff
        xbot_last_ref_pos[i] = DEG2RAD(refs[i]);
    }
    return success;
}

bool XBot::dev::XBotMotionControl::getRefPosition(const int j, double* ref)
{
    *ref = (double) RAD2DEG(xbot_last_ref_pos[j]);
    return true;
}

bool XBot::dev::XBotMotionControl::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    bool success = true;
    for( int i = 0; i < n_joint; i++ ) {
        success = getRefPosition(joints[i], &refs[i]) && success;
    }
    return success;
}

bool XBot::dev::XBotMotionControl::getRefPositions(double* refs)
{
    bool success = true;
    for( int i = 0; i < joints_num; i++ ) {
        success = getRefPosition(i, &refs[i]) && success;
    }
    return success;
}


// TBD don't do this: call Alberto Cardellino!!!!!
bool XBot::dev::XBotMotionControl::setPositionDirectMode()
{
    return false;
}


////////////////////// AxisInfo Interface

bool XBot::dev::XBotMotionControl::getAxisName(int axis, yarp::os::ConstString& name)
{
    name = _robot->getChainMap().at(chain_name)->getJointName(axis);
    return true;
}

bool XBot::dev::XBotMotionControl::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    // HACK TBD get the array from the robot_model
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}


////////////////////// ControlLimits Interface

bool XBot::dev::XBotMotionControl::getLimits(int axis, double* min, double* max)
{
    double actual_min = 0.0;
    double actual_max = 0.0;
    
    _robot->getJointByID(Yid_to_Jid.at(axis))->getJointLimits(actual_min, actual_max);
        
    *min = RAD2DEG(actual_min);
    *max = RAD2DEG(actual_max);

    
    DPRINTF("joint %d --> min: %f - max: %f\n", Yid_to_Jid.at(axis), *min, *max);
    
    return true;
}


bool XBot::dev::XBotMotionControl::setLimits(int axis, double min, double max)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getVelLimits(int axis, double* min, double* max)
{
    double actual_min = 0.0;
    double actual_max = 0.0;
    
    _robot->getJointByID(Yid_to_Jid.at(axis))->getVelocityLimit(actual_max);
        
    *min = RAD2DEG(actual_min);
    *max = RAD2DEG(actual_max);
    
    return true;
}

bool XBot::dev::XBotMotionControl::setVelLimits(int axis, double min, double max)
{
    return false;
}


/////////// CONTROL MODE INTERFACE 

bool XBot::dev::XBotMotionControl::getControlMode(int j, int* v)
{
    *v = controlMode[j];
    return true;
}

bool XBot::dev::XBotMotionControl::getControlModes(const int n_joint, const int* joints, int* modes)
{
    for( int i = 0; i < n_joint; i++ ) {
        modes[i] = controlMode[joints[i]];
    }
    return true;
}

bool XBot::dev::XBotMotionControl::getControlModes(int* v)
{
    for( int i = 0; i < joints_num; i++ ) {
        v[i] = controlMode[i];
    }
    return true;
}

bool XBot::dev::XBotMotionControl::setControlMode(const int j, const int mode)
{
    bool set_success = false;
    switch(mode)
    {
        case VOCAB_CM_IDLE:
        case VOCAB_CM_FORCE_IDLE:
        {
            set_success = false; //TBD check how to do it
            if( set_success ) {
                // set the control mode
                controlMode[j] = VOCAB_CM_IDLE;
            }
            else {
                std::cout << "ERROR : cannot proceed, setting IDLE mode failed " << std::endl;
            }
        }
        break;

        // POSITION DIRECT      
        case VOCAB_CM_POSITION_DIRECT:
//         case VOCAB_CM_POSITION:
        {
            // TBD set the pos_ref equal to actual pos

            // STIFF INTERACTION
            if( interactionMode[j] == VOCAB_IM_STIFF ) {
                // TBD set the control algorithms read from config file
                set_success = true; //TBD check how to do it
            }
            // COMPLIANT INTERACTION
            else if( interactionMode[j] == VOCAB_IM_COMPLIANT ) {
                // set impedance mode
                set_success = false; //TBD check how to do it
            }
            
            // POS DIRECT CONTROL MODE
            if( set_success ) {
                // set the control mode
                controlMode[j] = VOCAB_CM_POSITION_DIRECT;
            }
            else {
                std::cout << "ERROR : cannot proceed, cannot go to " << yarp::os::Vocab::decode(mode) << std::endl;
            }
        }
        break;
        default:
            std::cout << "ERROR : the following control mode is not implemented -> " << yarp::os::Vocab::decode(mode) << std::endl;
    }
    
    return set_success;
}

bool XBot::dev::XBotMotionControl::setControlModes(const int n_joint, const int* joints, int* modes)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setControlModes(int* modes)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setImpedancePositionMode(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setImpedanceVelocityMode(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setOpenLoopMode(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setPositionMode(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorqueMode(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setVelocityMode(int j)
{
    return false;
}




/////////// INTERACTION MODE INTERFACE 

bool XBot::dev::XBotMotionControl::getInteractionMode(int j, yarp::dev::InteractionModeEnum* _mode)
{
    *_mode = interactionMode[j];     
    return true;
}

bool XBot::dev::XBotMotionControl::getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    for( int i = 0; i < n_joints; i++ ) {
        modes[i] = interactionMode[joints[i]];
    }
    return true;
}

bool XBot::dev::XBotMotionControl::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    for( int i = 0; i < joints_num; i++ ) {
        modes[i] = interactionMode[i];
    }
    return true;
}


bool XBot::dev::XBotMotionControl::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return true;
}

bool XBot::dev::XBotMotionControl::setInteractionMode(int j, yarp::dev::InteractionModeEnum _mode)
{
    return true;
}

bool XBot::dev::XBotMotionControl::setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return true;
}




/////////// POSITION CONTROL INTERFACE 

bool XBot::dev::XBotMotionControl::checkMotionDone(const int n_joint, const int* joints, bool* flags)
{
    *flags = true;
    return true; // NOTE for PositionDirect
}

bool XBot::dev::XBotMotionControl::checkMotionDone(int j, bool* flag)
{
    *flag = true;
    return true; // NOTE for PositionDirect
}

bool XBot::dev::XBotMotionControl::checkMotionDone(bool* flag)
{
    *flag = true;
    return true; // NOTE for PositionDirect
}

bool XBot::dev::XBotMotionControl::getRefAcceleration(int j, double* acc)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefAccelerations(const int n_joint, const int* joints, double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefAccelerations(double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefSpeed(int j, double* ref)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefSpeeds(const int n_joint, const int* joints, double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefSpeeds(double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::positionMove(const int n_joint, const int* joints, const double* refs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::positionMove(const double* refs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::positionMove(int j, double ref)
{
    return false;
}

bool XBot::dev::XBotMotionControl::relativeMove(const int n_joint, const int* joints, const double* deltas)
{
    return false;
}

bool XBot::dev::XBotMotionControl::relativeMove(const double* deltas)
{
    return false;
}

bool XBot::dev::XBotMotionControl::relativeMove(int j, double delta)
{
    return false;
}

// TBD don't do this: call Alberto Cardellino!!!!!
bool XBot::dev::XBotMotionControl::setPositionMode()
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefAcceleration(int j, double acc)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefAccelerations(const int n_joint, const int* joints, const double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefAccelerations(const double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefSpeed(int j, double sp)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefSpeeds(const int n_joint, const int* joints, const double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefSpeeds(const double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::stop(const int n_joint, const int* joints)
{
    return false;
}

bool XBot::dev::XBotMotionControl::stop()
{
    return false;
}

bool XBot::dev::XBotMotionControl::stop(int j)
{
    return false;
}


/////////// VELOCITY CONTROL INTERFACE 

bool XBot::dev::XBotMotionControl::getVelPid(int j, Pid* pid)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getVelPids(Pid* pids)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setVelocityMode()
{
    return false;
}

bool XBot::dev::XBotMotionControl::setVelPid(int j, const Pid& pid)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setVelPids(const Pid* pids)
{
    return false;
}

bool XBot::dev::XBotMotionControl::velocityMove(const double* sp)
{
    return false;
}

bool XBot::dev::XBotMotionControl::velocityMove(int, double)
{
    return false;
}

bool XBot::dev::XBotMotionControl::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    return false;
}


/////////// TORQUE CONTROL INTERFACE 

bool XBot::dev::XBotMotionControl::getTorque(int j, double* t)
{
    *t = _robot->getChainMap().at(chain_name)->getJointEffort(j);
    return true;
}

bool XBot::dev::XBotMotionControl::getTorques(double* t)
{
    bool ret = true;
    for( int i = 0; i < joints_num; i++ ) {
        ret = getTorque(i, &t[i]) && ret;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::getTorqueError(int j, double* err)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorqueErrorLimit(int j, double* limit)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorqueErrorLimits(double* limits)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorqueErrors(double* errs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorquePid(int j, Pid* pid)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorquePidOutput(int j, double* out)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorquePidOutputs(double* outs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorquePids(Pid* pids)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorqueRange(int j, double* min, double* max)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getTorqueRanges(double* min, double* max)
{
    return false;
}

bool XBot::dev::XBotMotionControl::disableTorquePid(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::enableTorquePid(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getBemfParam(int j, double* bemf)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorTorqueParams(int j, MotorTorqueParameters* params)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefTorque(int j, double* t)
{
    *t = _robot->getChainMap().at(chain_name)->getEffortReference(j);
    return true;
}

bool XBot::dev::XBotMotionControl::getRefTorques(double* t)
{
    return false;
}

bool XBot::dev::XBotMotionControl::resetTorquePid(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setBemfParam(int j, double bemf)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setMotorTorqueParams(int j, const MotorTorqueParameters params)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefTorque(int j, double t)
{
    _robot->getChainMap().at(chain_name)->setEffortReference(j, t);
    return true;
}

bool XBot::dev::XBotMotionControl::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefTorques(const double* t)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorqueErrorLimit(int j, double limit)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorqueErrorLimits(const double* limits)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorqueMode()
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorqueOffset(int j, double v)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorquePid(int j, const Pid& pid)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setTorquePids(const Pid* pids)
{
    return false;
}



bool XBot::dev::XBotMotionControl::getNumberOfMotorEncoders(int* num)
{
    *num = joints_num;
    return true;
}

bool XBot::dev::XBotMotionControl::getMotorEncoder(int m, double* v)
{
    *v =  RAD2DEG(_robot->getChainMap().at(chain_name)->getMotorPosition(m));
    return true;
}

bool XBot::dev::XBotMotionControl::getMotorEncoders(double* encs)
{
    bool ret = true;
    for( int i = 0; i < joints_num; i++ ) {
        ret = getMotorEncoder(i, &encs[i]) && ret;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderTimed(int m, double* enc, double* stamp)
{
    *stamp = yarp::os::Time::now();
    *enc =  RAD2DEG(_robot->getChainMap().at(chain_name)->getMotorPosition(m));
    return true;
}

bool XBot::dev::XBotMotionControl::getMotorEncodersTimed(double* encs, double* stamps)
{
    bool ret = true;
    double stamp = yarp::os::Time::now();
    for( int i = 0; i < joints_num; i++ ) {
        encs[i] = RAD2DEG(_robot->getChainMap().at(chain_name)->getMotorPosition(i));
        stamps[i] = stamp;
    }
    return ret;
}

bool XBot::dev::XBotMotionControl::resetMotorEncoder(int m)
{
    return false;
}

bool XBot::dev::XBotMotionControl::resetMotorEncoders()
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderAcceleration(int m, double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderAccelerations(double* accs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderSpeed(int m, double* sp)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderSpeeds(double* spds)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setMotorEncoder(int m, const double val)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setMotorEncoders(const double* vals)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMotorEncoderCountsPerRevolution(int m, double* v)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    return false;
}


bool XBot::dev::XBotMotionControl::disableAmp(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::enableAmp(int j)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getAmpStatus(int* st)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getCurrent(int j, double* val)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getAmpStatus(int j, int* v)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getCurrents(double* vals)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getMaxCurrent(int j, double* v)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getPWM(int j, double* val)
{
    return true;
}

bool XBot::dev::XBotMotionControl::setMaxCurrent(int j, double v)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getDutyCycle(int m, double* val)
{
    return true;
}

bool XBot::dev::XBotMotionControl::getDutyCycles(double* vals)
{
    return true;
}

bool XBot::dev::XBotMotionControl::getNumberOfMotors(int* number)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefDutyCycle(int m, double* ref)
{
    return false;
}

bool XBot::dev::XBotMotionControl::getRefDutyCycles(double* refs)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefDutyCycle(int m, double ref)
{
    return false;
}

bool XBot::dev::XBotMotionControl::setRefDutyCycles(const double* refs)
{
    return false;
}










XBot::dev::XBotMotionControl::~XBotMotionControl() 
{

}