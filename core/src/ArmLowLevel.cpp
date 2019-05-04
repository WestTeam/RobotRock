// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>

#include <QThread>
#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmLowLevel.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

#define Z_DISABLED


ArmLowLevel::ArmLowLevel()
    : _attached( false )
    , _initOk( false )
    , _smartServo {nullptr,nullptr,nullptr}
{

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

    }

}


bool ArmLowLevel::init(
        const Hal::Ptr& hal,
        ItemRegister::Ptr vacuumPwm,
        ItemRegister::Ptr vacuumValve,
        uint32_t* distanceMmPtr,
        uint8_t upperArmProtocol,
        uint8_t upperArmbusId,
        uint8_t lowerArmProtocol,
        uint8_t lowerArmbusId,
        uint8_t wristProtocol,
        uint8_t wristbusId)
{
    bool ret = true;
    _attached = false;

    _hal = hal;

    _vacuumPwm = vacuumPwm;
    _vacuumValve = vacuumValve;

    _distanceMmPtr = distanceMmPtr;


    try {
        _smartServo[ARM_LL_SERVO_UPPER_ARM] = new SmartServo("Upper ARM protocol:"+ QString::number(upperArmProtocol) + " bus:" + QString::number(upperArmbusId));
        _smartServo[ARM_LL_SERVO_UPPER_ARM]->attach(hal,upperArmProtocol,upperArmbusId);

        _smartServo[ARM_LL_SERVO_LOWER_ARM] = new SmartServo("Lower ARM protocol:"+ QString::number(lowerArmProtocol) + " bus:" + QString::number(lowerArmbusId));
        _smartServo[ARM_LL_SERVO_LOWER_ARM]->attach(hal,lowerArmProtocol,lowerArmbusId);

        _smartServo[ARM_LL_SERVO_WRIST]     = new SmartServo("Wrist protocol:"+ QString::number(wristProtocol) + " bus:" + QString::number(wristbusId));
        _smartServo[ARM_LL_SERVO_WRIST]->attach(hal,wristProtocol,wristbusId);


        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            _smartServo[i]->setRawWrite8(DYNAMIXEL_REGS_P,128);
            _smartServo[i]->setRawWrite8(DYNAMIXEL_REGS_I,16);
        }

        _attached = true;
    } catch (...)
    {

        for (int i=ARM_LL_SERVO_UPPER_ARM;i<=ARM_LL_SERVO_WRIST;i++)
        {
            try {
                if (_smartServo[i] != nullptr)
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
            _smartServo[i]->setEnable(false,true);
            _smartServo[i]->setPosition(false,false,1024/2);

        } catch (...)
        {
            tDebug( LOG ) << "ArmLowLevel: Init -> Servo Set Init Position (512) Error" << i;
            ret = false;
            goto endfunction;
        }
    }

#ifndef Z_DISABLED
    // we now have to init the Z axe
    // first step: INIT the PID module

    // we disable PID during config
    _hal->_pidCustomEnable.write( 0 );

    // we get sw control of the PID
    _hal->_pidCustomOverride.write( 1 );

    // to check if we need to invert or not
    _hal->_pidCustomInverted.write( 0 );

    // we set coefs
    _hal->_pidCustomKp.write( (float)100.0 );
    _hal->_pidCustomKi.write( (float)0.0 );
    _hal->_pidCustomKd.write( (float)0.0 );

    // we set speed, acc and output saturation
    _hal->_pidCustomSpeed.write( (float)10.0 );
    _hal->_pidCustomAcceleration.write( (float)0.001 );
    _hal->_pidCustomSaturation.write( 15000 );



    // second step: find the Z reference (UP)
#define CALIBRATION_Z_FAKE_REF 666666
#define CALIBRATION_Z_STEP 100
#define CALIBRATION_Z_TIMEOUT 1000
#define CALIBRATION_Z_TIMEOUT_STEP 10

    // before finding the reference incrementally, we verify if we are "on" the ref point
    int32_t initialRef;
    int32_t currentPos;

    {
        bool refAway = false;
        uint8_t refAwayRetryCount = 3;

        do {

            _hal->_pidCustomLastReference.write(CALIBRATION_Z_FAKE_REF);
            QThread::msleep( 10 );
            initialRef = _hal->_pidCustomLastReference.read< int32_t >();
            currentPos = _hal->_pidCustomPosition.read< int32_t >();

            int32_t initPos = currentPos;

            if (initialRef != CALIBRATION_Z_FAKE_REF)
            {
                // it means we are on the ref point, so we need to move away
                initPos -= CALIBRATION_Z_STEP*20;
            } else {
                // OK!
                refAway = true;
            }

            _hal->_pidCustomEnable.write( 1 );
            _hal->_pidCustomTarget.write( initPos );
            QThread::msleep( CALIBRATION_Z_TIMEOUT );

        } while (refAway == false && refAwayRetryCount--);

        if (!refAway)
        {
            tWarning( LOG ) << "ArmLowLevel: Init -> Z Calibration: Ref Away issue for intialisation" << initialRef << currentPos;
            ret = false;
            goto endfunction;
        }

        bool refFound = false;


        initialRef = _hal->_pidCustomLastReference.read< int32_t >();
        do {

            currentPos = _hal->_pidCustomPosition.read< int32_t >();
            int32_t targetPos = currentPos + CALIBRATION_Z_STEP;
            _hal->_pidCustomTarget.write( targetPos );

            int32_t timeoutMs = CALIBRATION_Z_TIMEOUT;
            do {
                QThread::msleep( CALIBRATION_Z_TIMEOUT_STEP );
                timeoutMs -= CALIBRATION_Z_TIMEOUT_STEP;

                // if we have moved more than half of the target
                if ((targetPos-_hal->_pidCustomPosition.read< int32_t >()) < CALIBRATION_Z_STEP / 2)
                    break;
            } while (timeoutMs > 0);

            // check ref
            int32_t currentRef = _hal->_pidCustomLastReference.read< int32_t >();
            currentPos =  _hal->_pidCustomPosition.read< int32_t >();

            if (currentRef != initialRef)
            {
                // new ref found
                tDebug( LOG ) << "ArmLowLevel: Init -> Z Calibration: New Ref Found" << currentRef;

                if (timeoutMs < 0)
                {
                    // good, we have our reference point :)

                    // we set the target to the ref point to stop trying to reach
                    _refZ = currentRef;
                    setZ(getZ());
                    //_hal->_pidCustomTarget.write( currentRef );


                    // we save the currentRef
                    refFound = true;
                }

            } else {
                if (timeoutMs < 0)
                {
                    // if we are here, it means possiblity 2 things:
                    // 1: reference sensor is not working
                    // 2: there is an issue in the system

                    // either case, we cannot continue
                    tWarning( LOG ) << "ArmLowLevel: Init -> Z Calibration: Motor Blocked without reaching reference point (initRef/CurRef/CurPos)" << initialRef << currentRef << currentPos;

                    ret = false;
                    goto endfunction;
                }
            }

        } while (refFound == false);

        _initOk = true;
    }
#endif

endfunction:

    return ret;
}


/*

    bool init(
        const Hal::Ptr& hal,
        ItemRegister::Ptr vacuumPwm,
        ItemRegister::Ptr vacuumValve,
        uint32_t* distanceMmPtr,
        uint8_t upperArmProtocol,
        uint8_t upperArmbusId,
        uint8_t lowerArmProtocol,
        uint8_t lowerArmbusId,
        uint8_t wristProtocol,
        uint8_t wristbusId
    );

    void disable();

    bool isAttached() const;

    // general
    void disable();

    // Motor Z
    void enableZ(bool enable);
    void setZ(double mmAbs);
    double getZ();
    bool waitZTargetOk(double timeoutMs = 0);
    void setZSpeed(double speed);
    void setZSAcc(double acc);

    // Servos
    void enableServo(enum ArmLowLevelLeg, bool enable);
    void setServoPos(enum ArmLowLevelLeg, double angleDegs);
    void setServosPos(double angleDegs1, double angleDegs2, double angleDegs3) ;
    double getServoPos(enum ArmLowLevelLeg);
    bool waitServoTargetOk(enum ArmLowLevelLeg, double timeoutMs);
    bool waitServosTargetOk(double timeoutMs);

    // Vacuum
    setVacuumPower(float percentage) ;
    setVacuumValve(bool enable);

    // Distance
    double getDistance();
*/


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
    _hal->_pidCustomEnable.write( (uint8_t)enable );
#endif
}
void ArmLowLevel::setZ(double mmAbs)
{

#define Z_ENCODER_TICK_PER_MM (128.0/4.0)
#define Z_REF_POS_ABS_MM (320.0)
#define Z_MIN_POS_MM (40.0)

#ifndef Z_DISABLED

    if (mmAbs < Z_MIN_POS_MM)
        mmAbs = Z_MIN_POS_MM;

    if (mmAbs > Z_REF_POS_ABS_MM)
        mmAbs = Z_REF_POS_ABS_MM;

    // convert mmAbs into ticks
    int32_t target = _refZ-(int32_t)((Z_REF_POS_ABS_MM-mmAbs)*Z_ENCODER_TICK_PER_MM);

    _hal->_pidCustomTarget.write( target );
#endif

}
double ArmLowLevel::getZ()
{
    double ret = 0.0;

#ifndef Z_DISABLED
    int32_t currentPos = _hal->_pidCustomPosition.read< int32_t >();

    ret = Z_REF_POS_ABS_MM - ((double)(_refZ-currentPos))*1.0/Z_ENCODER_TICK_PER_MM;

    if (ret > Z_REF_POS_ABS_MM)
        ret = Z_REF_POS_ABS_MM;

    if (ret < Z_MIN_POS_MM)
        ret = Z_MIN_POS_MM;
#endif

    return ret;
}
bool ArmLowLevel::waitZTargetOk(double timeoutMs)
{
    bool ret = false;

#define WAIT_STEP_MS (10.0)
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
    _hal->_pidCustomSpeed.write( (float)pidSpeed );
}
void ArmLowLevel::setZAcc(double acc)
{
// acc in mm per second ^2
    double pidAcc = acc * Z_ENCODER_TICK_PER_MM / PID_FREQ_HZ;
    _hal->_pidCustomAcceleration.write( (float)pidAcc );
}


void ArmLowLevel::enableServo(enum ArmLowLevelLeg id, bool enable)
{
    try {
        if (enable)
            _smartServo[id]->enable();
        else
            _smartServo[id]->disable();
    } catch (...) {

    }
}
void ArmLowLevel::setServoPos(enum ArmLowLevelLeg id, double angleDegs)
{
#define SERVO_RANGE_DEG (300.0)
#define SERVO_TICK_PER_DEG (1024.0/SERVO_RANGE_DEG)
#define SERVO_OFFSET (1024.0/2.0)

    double pos = (SERVO_OFFSET+angleDegs*SERVO_TICK_PER_DEG);
    if (pos > 1023)
        pos = 1023;
    if (pos < 0)
        pos = 0;

    try {
        //tInfo( LOG ) << id << angleDegs << SERVO_OFFSET << angleDegs*SERVO_TICK_PER_DEG <<  pos << (uint16_t)pos;

        _smartServo[id]->setPosition(false,false,(uint16_t)pos);
    } catch (...) {

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

        tInfo( LOG ) << moving << timeoutMsLocal;

    } while (moving && timeoutMsLocal > 0);

    return !moving;
}

// Vacuum
void ArmLowLevel::setVacuumPower(float percentage)
{
    float max = 3.7/12.0*32768.0;
    float value = percentage/100.0*max;
    _vacuumPwm->write((uint16_t)value);
}
void ArmLowLevel::setVacuumValve(bool enable)
{
    _vacuumValve->write((uint8_t)enable);
}

// Distance
double ArmLowLevel::getDistance()
{
    return ((double)*_distanceMmPtr);
}

