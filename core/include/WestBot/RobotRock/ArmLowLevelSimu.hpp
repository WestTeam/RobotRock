// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ARM_LOW_LEVELSIMU_HPP_
#define WESTBOT_ROBOTROCK_ARM_LOW_LEVELSIMU_HPP_

#include "ArmLowLevel.hpp"
#include "ArmHighLevel.hpp"

namespace WestBot {
namespace RobotRock {

class TableObject
{
public:
    TableObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode);

    void reset();
    bool isCatchable();
    bool isCatched();

    double getDistance(Pos3D wristPos, enum ArmHighLevelMode mode);

    void catchObject();
    void releaseObject(Pos3D wristPos, enum ArmHighLevelMode mode);

private:
    Pos3D _pos;
    bool _isCatchable;
    bool _isCatched;
    enum ArmHighLevelMode _mode;

    Pos3D _resetPos;
    bool _resetIsCatchable;
    enum ArmHighLevelMode _resetMode;
};

class ArmLowLevelTest: public ArmLowLevelBase
{
public:
    using Ptr = std::shared_ptr< ArmLowLevelTest >;

    ArmLowLevelTest(const Odometry::Ptr odometry, RobotPos armPos);

    ~ArmLowLevelTest();

    void resetObjects();

    void addObject(TableObject object);

    void addObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode);

    Pos3D getPos();

    void checkObjects();

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
    void setServosPos(double angleDegs1, double angleDegs2, double angleDegs3);
    double getServoPos(enum ArmLowLevelLeg id);
    bool waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs);
    bool waitServosTargetOk(double timeoutMs);

    // Vacuum
    void setVacuumPower(float percentage);
    void setVacuumValve(bool enable);

    // Distance
    double getDistance();

    bool isDistanceCoherent();

private:
    Odometry::Ptr _odometry;
    enum ArmHighLevelMode _mode;
    RobotPos _armPos;
    double _wristAngle;
    bool _zEnabled;
    double _zPosition;
    bool _servoEnable[3];
    double _servoPos[3];
    float _vacuumPower;
    bool _vacuumValve;

    double _curDistance;

    TableObject* _catchedObject;
    std::list<TableObject> _listObjects;
};

}
}

#endif // WESTBOT_ROBOTROCK_ARM_LOW_LEVEL_HPP_

