// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>

#include <QThread>
#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmLowLevel.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

//#define Z_DISABLED


ArmLowLevel::ArmLowLevel()
    : _attached( false )
    , _initOk( false )
    , _smartServo {nullptr,nullptr,nullptr}
{
    _refInverted = 1;
    _vaccumEnabled = false;
    _pid = nullptr;
}

ArmLowLevel::~ArmLowLevel()
{

    tDebug( LOG ) << "ArmLowLevel destructor engaged";

    disable();

    if (_attached)
    {
        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            try {
                delete _smartServo[i];
            } catch (...)
            {

            }
        }

        //if (_pid)
        //    delete _pid;
    }

}


bool ArmLowLevel::init(
        const Hal::Ptr& hal,
        const ItemRegister::Ptr& pidFirstReg,
        bool pidInputInverted,
        const ItemRegister::Ptr& vacuumPwm,
        const ItemRegister::Ptr& vacuumValve,
        Vl6180x* distanceSensors,
        uint32_t* distanceMmPtr,
        uint8_t upperArmProtocol,
        uint8_t upperArmbusId,
        uint8_t lowerArmProtocol,
        uint8_t lowerArmbusId,
        uint8_t wristProtocol,
        uint8_t wristbusId,
        bool wristInverted,
        double zOffset)
{
    bool ret = true;
    _attached = false;
    _wristInverted = wristInverted;
    _zOffset = zOffset;



    if (pidInputInverted)
        _refInverted = -1;
    else
        _refInverted = 1;

    //Memory _pidlayer = pidFirstReg->layer();

    _hal = hal;

    _vacuumPwm = vacuumPwm;
    setVacuumPower(0.0);

    _vacuumValve = vacuumValve;

    _distanceMmPtr = distanceMmPtr;

    _distanceSensors = distanceSensors;

    try {
        _smartServo[ARM_LL_SERVO_UPPER_ARM] = new SmartServo("Upper ARM protocol:"+ QString::number(upperArmProtocol) + " bus:" + QString::number(upperArmbusId));
        _smartServo[ARM_LL_SERVO_UPPER_ARM]->attach(hal,upperArmProtocol,upperArmbusId);

        _smartServo[ARM_LL_SERVO_LOWER_ARM] = new SmartServo("Lower ARM protocol:"+ QString::number(lowerArmProtocol) + " bus:" + QString::number(lowerArmbusId));
        _smartServo[ARM_LL_SERVO_LOWER_ARM]->attach(hal,lowerArmProtocol,lowerArmbusId);

        _smartServo[ARM_LL_SERVO_WRIST]     = new SmartServo("Wrist protocol:"+ QString::number(wristProtocol) + " bus:" + QString::number(wristbusId));
        _smartServo[ARM_LL_SERVO_WRIST]->attach(hal,wristProtocol,wristbusId);


        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            _smartServo[i]->setRawWrite8(DYNAMIXEL_REGS_P,80);
            _smartServo[i]->setRawWrite8(DYNAMIXEL_REGS_I,0);

            _smartServo[i]->setRawWrite16(DYNAMIXEL_REGS_MAX_TORQUE_L,512);

            //_smartServo[i]->setRawWrite8(DYNAMIXEL_REGS_PUNCH,32);

        }

        _attached = true;
    } catch (...)
    {

        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            try {
                if (_smartServo[i] != nullptr)
                     tDebug( LOG ) << "ArmLowLevel: Init -> Due to error, deleting servo " << i;
                    delete _smartServo[i];
            } catch (...)
            {
                tDebug( LOG ) << "ArmLowLevel: Init -> Servo Constructor Error" << i;
            }
        }

        ret = false;
        goto endfunction;
    }

    // we set the servo position in "safe state"
    for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
    {
        try {
            tDebug( LOG ) << "ArmLowLevel: Init -> Servo " << i;

            {
                _smartServo[i]->setEnable(false,true);
                _smartServo[i]->setPositionAndSpeed(false,false,1024/2,400);
            }

            //QThread::msleep(500);

        } catch (...)
        {
            tDebug( LOG ) << "ArmLowLevel: Init -> Servo Set Init Position (512) Error" << i;
            ret = false;
            goto endfunction;
        }
    }



#ifndef Z_DISABLED
    _pid = new Pid(pidFirstReg->_layer,pidFirstReg->offset());

    // we now have to init the Z axe
    // first step: INIT the PID module

    //_hal->_pidCustomTarget.write( _hal->_pidCustomPosition.read< int32_t >() );
    _pid->setTarget(_pid->getInput());


    // we disable PID during config
    //_hal->_pidCustomEnable.write( 0 );
    _pid->setEnable(false);



    // we get sw control of the PID
    //_hal->_pidCustomOverride.write( 1 );
    _pid->setOverride(true);

    // to check if we need to invert or not
    //_hal->_pidCustomInverted.write( 0 );
    _pid->setInverted(false);

    // we set coefs
    //_hal->_pidCustomKp.write( (float)300.0 );
    //_hal->_pidCustomKi.write( (float)0.0 );
    //_hal->_pidCustomKd.write( (float)0.0 );
    _pid->setKp(50.0);
    _pid->setKi(0.0);
    _pid->setKd(0.0);

    // we set speed, acc and output saturation
    //_hal->_pidCustomSpeed.write( (float)40.0 );
    //_hal->_pidCustomAcceleration.write( (float)0.010 );
    //_hal->_pidCustomSaturation.write( 10000 );
    _pid->setSpeed(40.0);
    _pid->setAcceleration(0.010);
    _pid->setSaturation(12000);



    // second step: find the Z reference (UP)
#define CALIBRATION_Z_FAKE_REF 666666
#define CALIBRATION_Z_STEP 600
#define CALIBRATION_Z_TIMEOUT 1000
#define CALIBRATION_Z_TIMEOUT_STEP 100

    // before finding the reference incrementally, we verify if we are "on" the ref point
    int32_t initialRef;
    int32_t currentPos;

    _pid->setEnable(true);

    {

        bool refFound = false;

        do {

            currentPos = _pid->getInput();//_hal->_pidCustomPosition.read< int32_t >();

            int32_t targetPos = currentPos + CALIBRATION_Z_STEP*_refInverted;
            //_hal->_pidCustomTarget.write( targetPos );
            _pid->setTarget(targetPos);

            //tDebug(LOG) << "Loop 2" << currentPos << targetPos << _hal->_pidCustom1Enable.read<uint8_t>() << _hal->_pidCustom1Output.read<int32_t>() << _pid->getOutput();


            int32_t timeoutMs = CALIBRATION_Z_TIMEOUT;
            do {
                QThread::msleep( CALIBRATION_Z_TIMEOUT_STEP );
                timeoutMs -= CALIBRATION_Z_TIMEOUT_STEP;

                //tDebug(LOG) << "Loop 3" << _pid->getInput() << targetPos;


                // if we have moved more than half of the target
                if (abs(targetPos-_pid->getInput()) < CALIBRATION_Z_STEP / 2)
                    break;
            } while (timeoutMs > 0);

            // check ref
            //int32_t currentRef = _pid->getReference();//_hal->_pidCustomLastReference.read< int32_t >();
            currentPos =  _pid->getInput();//_hal->_pidCustomPosition.read< int32_t >();

            /*if (currentRef != initialRef)
            {
                // new ref found
                tDebug( LOG ) << "ArmLowLevel: Init -> Z Calibration: New Ref Found" << currentRef;

                if (timeoutMs <= 0)
                {
                    // good, we have our reference point :)

                    // we set the target to the ref point to stop trying to reach
                    _refZ = currentRef;
                    setZ(getZ());
                    //_hal->_pidCustomTarget.write( currentRef );


                    // we save the currentRef
                    refFound = true;
                }

            } else */
            {
                if (timeoutMs <= 0)
                {
                    // if we are here, it means possiblity 2 things:
                    // 1: reference sensor is not working
                    // 2: there is an issue in the system

                    // either case, we cannot continue
                    tDebug( LOG ) << "ArmLowLevel: Init -> Z Calibration: Motor Blocked at (CurPos)" << currentPos;

                    _refZ = currentPos;

                    tDebug( LOG ) << "ArmLowLevel: Init -> Z Calibration: Current Z" << getZ();

                    //_hal->_pidCustomSaturation.write( 18000 );

                    setZ(getZ()-20.0);

                    _pid->setSaturation(18000);


                    //_hal->_pidCustomTarget.write( currentPos-CALIBRATION_Z_STEP*10 );

                    refFound = true;
                    //ret = false;
                    //goto endfunction;
                }
            }

        } while (refFound == false);

        _initOk = true;


    }
#endif
endfunction:
    tDebug( LOG ) << "ArmLowLevel: Init Done" << ret;

    return ret;
}


void ArmLowLevel::disable()
{
    if (_attached)
    {
        setVacuumPower(0.0);
        setVacuumValve(true);

        enableZ(false);

        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            enableServo((enum ArmLowLevelLeg)i,false);
        }
    }
    tInfo( LOG ) <<  "ArmLowLevel Disabled";
}

bool ArmLowLevel::isAttached() const
{
    return _attached;
}


void ArmLowLevel::enableZ(bool enable)
{
#ifndef Z_DISABLED
    //_hal->_pidCustomEnable.write( (uint8_t)enable );
    _pid->setEnable(enable);
#endif
}
void ArmLowLevel::setZ(double mmAbs)
{

#define Z_ENCODER_TICK_PER_MM (1024.0/4.0*0.97)
#define Z_REF_POS_ABS_MM (283.0)
#define Z_MIN_POS_MM (45.0)
//37.6 max
//280.0
#ifndef Z_DISABLED

    if (mmAbs < Z_MIN_POS_MM)
        mmAbs = Z_MIN_POS_MM;

    if (mmAbs > (Z_REF_POS_ABS_MM+_zOffset))
        mmAbs = (Z_REF_POS_ABS_MM+_zOffset);

    _targetZMmAbs = mmAbs;

    // convert mmAbs into ticks
    int32_t target = _refZ-((int32_t)(((Z_REF_POS_ABS_MM+_zOffset)-mmAbs)*Z_ENCODER_TICK_PER_MM))*_refInverted;

    //_hal->_pidCustomTarget.write( target );
    _pid->setTarget(target);

    tDebug(LOG) << mmAbs << target << _refZ << (Z_REF_POS_ABS_MM+_zOffset)-mmAbs << ((int32_t)(((Z_REF_POS_ABS_MM+_zOffset)-mmAbs)*Z_ENCODER_TICK_PER_MM));
#endif

}
double ArmLowLevel::getZ()
{
    double ret = 0.0;

#ifndef Z_DISABLED
    int32_t currentPos = _pid->getInput();//_hal->_pidCustomPosition.read< int32_t >();

    ret = (Z_REF_POS_ABS_MM+_zOffset) - ((double)((_refZ-currentPos)*_refInverted))*1.0/Z_ENCODER_TICK_PER_MM;

    if (ret > (Z_REF_POS_ABS_MM+_zOffset))
        ret = (Z_REF_POS_ABS_MM+_zOffset);

    if (ret < Z_MIN_POS_MM)
        ret = Z_MIN_POS_MM;
#endif

    //tDebug(LOG) << ret << currentPos << _refZ-currentPos << ((double)((_refZ-currentPos)*_refInverted))*1.0/Z_ENCODER_TICK_PER_MM;


    return ret;
}
bool ArmLowLevel::waitZTargetOk(double timeoutMs)
{
    bool ret = false;

#define WAIT_STEP_MS (100.0)
#define WAIT_MARGIN_MM (5.0)
#define WAIT_MAX_MS (10000.0)

#ifndef Z_DISABLED
    double timeoutMsLocal = timeoutMs;
    if (timeoutMsLocal == 0)
        timeoutMsLocal = WAIT_MAX_MS;
    do {
        QThread::msleep( WAIT_STEP_MS );
        timeoutMsLocal -= WAIT_STEP_MS;

        double diff = _targetZMmAbs-getZ();

        //tInfo( LOG ) << "wait target z" << _targetZMmAbs << getZ() << diff << timeoutMsLocal;


        // if we have moved more than half of the target
        if (abs(diff) <= WAIT_MARGIN_MM)
            ret = true;

    } while (!ret && timeoutMsLocal > 0);
#else
    ret = true;
#endif
    return ret;
}
void ArmLowLevel::setZSpeed(double speed)
{
// speed in mm per second
#define PID_FREQ_HZ (10000.0)

    double pidSpeed = speed * Z_ENCODER_TICK_PER_MM / PID_FREQ_HZ;
    //_hal->_pidCustomSpeed.write( (float)pidSpeed );
    _pid->setSpeed((float)pidSpeed);
}
void ArmLowLevel::setZAcc(double acc)
{
// acc in mm per second ^2
    double pidAcc = acc * Z_ENCODER_TICK_PER_MM / PID_FREQ_HZ;
    //_hal->_pidCustomAcceleration.write( (float)pidAcc );
    _pid->setAcceleration((float)pidAcc);

}


void ArmLowLevel::enableServo(enum ArmLowLevelLeg id, bool enable)
{    
    int retry_count = 10;
    bool ok = false;
    do {

        try {
            {
                if (enable)
                    _smartServo[id]->enable();
                else
                    _smartServo[id]->disable();
            }
            ok = true;
        } catch (...) {
            tDebug(LOG) << "SmartServo: enableServo exception" << id << enable;
        }
    } while (retry_count-- && !ok);

}
void ArmLowLevel::setServoPos(enum ArmLowLevelLeg id, double angleDegs)
{
    uint8_t retry_count = 10;


#define SERVO_RANGE_DEG (300.0)
#define SERVO_TICK_PER_DEG (1024.0/SERVO_RANGE_DEG)
#define SERVO_OFFSET (1024.0/2.0)

    if (id == ARM_LL_SERVO_WRIST && _wristInverted)
        angleDegs = -1.0*angleDegs;

    double pos = (SERVO_OFFSET+angleDegs*SERVO_TICK_PER_DEG);
    if (pos > 1023)
        pos = 1023;
    if (pos < 0)
        pos = 0;

    bool ok = false;
    do
    {
        try {
            //tInfo( LOG ) << id << angleDegs << SERVO_OFFSET << angleDegs*SERVO_TICK_PER_DEG <<  pos << (uint16_t)pos;
            uint16_t speed = 120;
            if (_vaccumEnabled)
            {
                if (id == ARM_LL_SERVO_UPPER_ARM)
                {
                    speed = 100;
                }
                if (id == ARM_LL_SERVO_LOWER_ARM)
                {
                    speed = 100;
                }
            }
            _smartServo[id]->setPositionAndSpeed(false,false,(uint16_t)pos,speed);
            ok = true;
        } catch (...) {
            tDebug(LOG) << "SmartServo: setPositionAndSpeed failure, (id/pos/speed/retry left)" << id << pos << retry_count;
        }
    } while (retry_count-- && !ok);

    if (!ok)
    {
        tWarning(LOG) << "SmartServo: setPositionAndSpeed FATAL (id/pos/speed)" << id << pos;
    }
}
void ArmLowLevel::setServosPos(double angleDegs1, double angleDegs2, double angleDegs3)
{
    setServoPos(ARM_LL_SERVO_UPPER_ARM,angleDegs1);
    setServoPos(ARM_LL_SERVO_LOWER_ARM,angleDegs2);
    setServoPos(ARM_LL_SERVO_WRIST,angleDegs3);
}
double ArmLowLevel::getServoPos(enum ArmLowLevelLeg id)
{
    uint16_t posRaw = _smartServo[id]->getPosition(true);
    double pos = ((double)(posRaw)-SERVO_OFFSET)/SERVO_TICK_PER_DEG;

    if (id == ARM_LL_SERVO_WRIST && _wristInverted)
        pos = -1.0*pos;

    return pos;
}
bool ArmLowLevel::waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs)
{
    double timeoutMsLocal = timeoutMs;
    bool moving = true;

    if (timeoutMsLocal == 0)
        timeoutMsLocal = WAIT_MAX_MS;
    do {
        QThread::msleep( WAIT_STEP_MS );
        timeoutMsLocal -= WAIT_STEP_MS;

        moving = _smartServo[id]->moving();

    } while (moving && timeoutMsLocal > 0);

    return !moving;
}
bool ArmLowLevel::waitServosTargetOk(double timeoutMs)
{
    double timeoutMsLocal = timeoutMs;
    bool moving = true;

    if (timeoutMsLocal == 0)
        timeoutMsLocal = WAIT_MAX_MS;
    do {
        QThread::msleep( WAIT_STEP_MS );
        timeoutMsLocal -= WAIT_STEP_MS;

        moving = _smartServo[ARM_LL_SERVO_UPPER_ARM]->moving();
        moving |= _smartServo[ARM_LL_SERVO_LOWER_ARM]->moving();
        moving |= _smartServo[ARM_LL_SERVO_WRIST]->moving();

        if (timeoutMsLocal < (timeoutMs / 4.0))
        {
            moving = false;
            for (int i=0;i<=ARM_LL_SERVO_WRIST;i++)
            {
                try {
                    double pos;
                    if (_smartServo[i]->moving())
                    {
                        uint16_t target = _smartServo[i]->getTarget();
                        uint16_t pos = _smartServo[i]->getPosition(true);

                        if (abs(target-pos) < 20*2)
                        {
                            tWarning(LOG) << "waitServosTargetOk:" << i << "is close to destination, seems ok (target/pos)" << target << pos;
                        } else {
                            try {
                                _smartServo[i]->disable();
                                QThread::msleep(200);
                                _smartServo[i]->enable();
                                _smartServo[i]->setPositionAndSpeed(false,false,target, 120);
                                QThread::msleep(500);

                            } catch (...) {
                            }

                            moving |= true;
                            tWarning(LOG) << "waitServosTargetOk:" << i << "seems really bloked (target/pos)" << target << pos;
                        }
                    }
                } catch (...) {
                    moving = true;
                    tWarning(LOG) << "waitServosTargetOk: exception for servo " << i;
                }
            }
/*
            tInfo( LOG ) << moving << timeoutMsLocal << _smartServo[ARM_LL_SERVO_UPPER_ARM]->moving() << _smartServo[ARM_LL_SERVO_LOWER_ARM]->moving() << _smartServo[ARM_LL_SERVO_WRIST]->moving();
            try {
                pos[0] =
                tInfo( LOG ) << moving << timeoutMsLocal << _smartServo[ARM_LL_SERVO_UPPER_ARM]->getPosition(true) << _smartServo[ARM_LL_SERVO_LOWER_ARM]->getPosition(true) << _smartServo[ARM_LL_SERVO_WRIST]->getPosition(true);
            } catch (...) {


            }*/
        }

    } while (moving && timeoutMsLocal > 0);


    tWarning(LOG) << "waitServosTargetOk: FATAL, servo still moving after timeout";

    return !moving;
}

// Vacuum
void ArmLowLevel::setVacuumPower(float percentage)
{
    float max = 3.7/12.0*32768.0;
    float value = percentage/100.0*max;
    if (value != 0)
        _vaccumEnabled = true;
    else
        _vaccumEnabled = false;
    _vacuumPwm->write((uint16_t)value);
}
void ArmLowLevel::setVacuumValve(bool enable)
{
    _vacuumValve->write((uint8_t)enable);
}

// Distance
double ArmLowLevel::getDistance()
{
    bool status[2];
    double distance[2];

    status[0] = _distanceSensors->status(0);
    status[1] = _distanceSensors->status(1);

    distance[0] = _distanceSensors->distance(0);
    distance[1] = _distanceSensors->distance(1);


    if (!status[0] || !status[1])
        tWarning( LOG ) << "getDistance status issues (status/distance)" << status[0] << distance[0] << status[1] << distance[1];

    if (status[0] && status[1])
    {
        if (distance[0] <= distance[1])
            return distance[0];
        else
            return distance[1];
    }
    if (status[0])
        return distance[0];
    else
        return distance[1];


    //return ((double)*_distanceMmPtr);
}

bool ArmLowLevel::isDistanceCoherent()
{
    double d1 = _distanceSensors->distance(0);
    double d2 = _distanceSensors->distance(1);

    double diff = abs(d1-d2);

    tDebug(LOG) << "isDistanceCoherent: d1/d2/diff" << d1 << d2 << diff;


    bool ret = true;

    if (_distanceSensors->status(0) && _distanceSensors->status(1))
    {
        if (diff > d1/2.0 || diff > d2/2.0)
            ret = false;
    }

    if (!ret)
        tWarning( LOG ) << "isDistanceCoherent: not coherent" << d1 << d2 << diff;

    return ret;
}
