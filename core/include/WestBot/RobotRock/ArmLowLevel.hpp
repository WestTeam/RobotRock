// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARM_LOW_LEVEL_HPP_
#define WESTBOT_ROBOTROCK_ARM_LOW_LEVEL_HPP_

#include <array>
#include <mutex>          // std::mutex, std::unique_lock
#include <queue>
#include <QMutex>

#include <QString>

#include "Hal.hpp"


#include <WestBot/RobotRock/SmartServo.hpp>
#include <WestBot/RobotRock/Vl6180x.hpp>
#include <WestBot/RobotRock/Pid.hpp>


namespace WestBot {
namespace RobotRock {


enum ArmLowLevelLeg { ARM_LL_SERVO_UPPER_ARM, ARM_LL_SERVO_LOWER_ARM, ARM_LL_SERVO_WRIST };


class ArmLowLevelBase
{
public:
    using Ptr = std::shared_ptr< ArmLowLevelBase >;

    // general
    virtual void disable() = 0;

    // Motor Z
    virtual void enableZ(bool enable) = 0;
    virtual void setZ(double mmAbs) = 0;
    virtual double getZ() = 0;
    virtual bool waitZTargetOk(double timeoutMs = 0) = 0;
    virtual void setZSpeed(double speed) = 0;
    virtual void setZAcc(double acc) = 0;

    // Servos
    virtual void enableServo(enum ArmLowLevelLeg id, bool enable) = 0;
    virtual void setServoPos(enum ArmLowLevelLeg id, double angleDegs) = 0;
    virtual void setServosPos(double angleDegs1, double angleDegs2, double angleDegs3) = 0;
    virtual double getServoPos(enum ArmLowLevelLeg id) = 0;
    virtual bool waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs) = 0;
    virtual bool waitServosTargetOk(double timeoutMs) = 0;

    // Vacuum
    virtual void setVacuumPower(float percentage) = 0;
    virtual void setVacuumValve(bool enable) = 0;

    // Distance
    virtual double getDistance() = 0;
    virtual bool isDistanceCoherent() = 0;

};


class ArmLowLevel: public ArmLowLevelBase
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.ArmLowLevel" );

    using Ptr = std::shared_ptr< ArmLowLevel >;


    ArmLowLevel();
    ~ArmLowLevel();

    bool init(
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
        double zOffset
    );

    bool isAttached() const;

    // general
    void disable();

    // Motor Z
    void enableZ(bool enable);
    void setZ(double mmAbs);
    double getZ();
    bool waitZTargetOk(double timeoutMs = 0);
    void setZSpeed(double speed);
    void setZAcc(double acc);

    // Servos
    void enableServo(enum ArmLowLevelLeg id, bool enable);
    void setServoPos(enum ArmLowLevelLeg id, double angleDegs);
    void setServosPos(double angleDegs1, double angleDegs2, double angleDegs3) ;
    double getServoPos(enum ArmLowLevelLeg id);
    bool waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs);
    bool waitServosTargetOk(double timeoutMs);

    // Vacuum
    void setVacuumPower(float percentage) ;
    void setVacuumValve(bool enable);

    // Distance
    double getDistance();
    bool isDistanceCoherent();


private:

    bool _attached;
    bool _initOk;

    bool _wristInverted;
    double _zOffset;

    Hal::Ptr _hal;
    ItemRegister::Ptr _vacuumPwm;
    ItemRegister::Ptr _vacuumValve;
    uint32_t* _distanceMmPtr;
    SmartServo* _smartServo[3];

    int32_t _refZ;
    int32_t _refInverted;
    double _targetZMmAbs;

    Vl6180x* _distanceSensors;

    Pid* _pid;

};

}
}

#endif // WESTBOT_ROBOTROCK_ARM_LOW_LEVEL_HPP_
