// Copyright (c) 2019 All Rights Reserved WestBot

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmLowLevelSimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testArmLowLevelSimu" )
}

TableObject::TableObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode)
{
    _resetPos = pos;
    _resetIsCatchable = isCatchable;
    _resetMode = mode;

    _pos = pos;
    _mode = mode;
    _isCatchable = isCatchable;
}

void TableObject::reset() {
    _isCatched = false;

    _pos = _resetPos;
    _mode = _resetMode;
    _isCatchable = _resetIsCatchable;
}

bool TableObject::isCatchable() {
    return _isCatchable;
}

bool TableObject::isCatched() {
    return _isCatched;
}

double TableObject::getDistance(Pos3D wristPos, enum ArmHighLevelMode mode)
{
    double dist = 255.0;
    if (mode == ARM_HL_MODE_VERTICAL && _mode == mode)
    {
        if (abs(wristPos.z - _pos.z) <= 5)
            dist = hypot(_pos.x-wristPos.x,_pos.y-wristPos.y);
    }

    if (mode == ARM_HL_MODE_HORIZONTAL && _mode == mode)
    {
        if (abs(wristPos.x - _pos.x) < 5 && abs(wristPos.y - _pos.y) < 5)
            dist = wristPos.z - _pos.z;
    }
    return dist;
}

void TableObject::catchObject()
{
    if (_isCatchable)
    {
        _isCatched = true;
        tInfo(LOG) << "TableObject: catched!" << _pos.x << _pos.y << _pos.z;
    }
}
void TableObject::releaseObject(Pos3D wristPos, enum ArmHighLevelMode mode)
{
    _pos = wristPos;
    _mode = mode;
    _isCatched = false;

    tInfo(LOG) << "TableObject: released!" << _pos.x << _pos.y << _pos.z << _mode;

}

ArmLowLevelTest::ArmLowLevelTest(const Odometry::Ptr odometry, RobotPos armPos)
{
    _mode = ARM_HL_MODE_HORIZONTAL;
    _armPos = armPos;
    _curDistance = 255.0;
    _odometry = odometry;
    _catchedObject = nullptr;
    tInfo(LOG) << "ArmLowLevelTest: Constructor";
}

ArmLowLevelTest::~ArmLowLevelTest()
{
    tInfo(LOG) << "ArmLowLevelTest: Destructor";

}

void ArmLowLevelTest::resetObjects()
{
    _catchedObject = nullptr;

    std::list<TableObject>::iterator it;
    for (it=_listObjects.begin(); it!=_listObjects.end(); ++it)
    {
        it->reset();
    }
}

void ArmLowLevelTest::addObject(TableObject object)
{
    _listObjects.push_back(object);
}

void ArmLowLevelTest::addObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode)
{
    _listObjects.push_back(TableObject(pos,isCatchable,mode));
}

Pos3D ArmLowLevelTest::getPos()
{
    Pos3D pos;

    double lenWrist = 70+35.0;

    pos.z = _zPosition;
    if (_servoPos[ARM_LL_SERVO_WRIST] <= -89.0)
    {
        _mode = ARM_HL_MODE_HORIZONTAL;
        pos.z -= 35.0;
        lenWrist -= 35.0;
    } else {
        _mode = ARM_HL_MODE_VERTICAL;
    }

    RobotPos rpos = _odometry->getPosition();

    RobotPos armPosAbs;

    double angleCenterArm = atan2(_armPos.y-0,_armPos.x-0);
    //  tInfo(LOG) << "LidarTest: angleCenterLidar " << angleCenterLidar;
    double distanceCenterArm = sqrt(_armPos.x*_armPos.x+_armPos.y*_armPos.y);

    // we recompute the real absolute position of the ARM center
    armPosAbs.x = rpos.x + distanceCenterArm * cos(rpos.theta+angleCenterArm);
    armPosAbs.y = rpos.y + distanceCenterArm * sin(rpos.theta+angleCenterArm);
    armPosAbs.theta = rpos.theta;

    tInfo(LOG) << "ArmHighLevelTest: getPos armPosAbs" << armPosAbs.x << armPosAbs.y << DEG(armPosAbs.theta);


    // next stage position compute
    RobotPos armLowerAbs;
    armLowerAbs.theta = armPosAbs.theta + RAD(_servoPos[ARM_LL_SERVO_UPPER_ARM]);
    armLowerAbs.x = armPosAbs.x + 70.0 * cos(armLowerAbs.theta);
    armLowerAbs.y = armPosAbs.y + 70.0 * sin(armLowerAbs.theta);

    tInfo(LOG) << "ArmHighLevelTest: getPos armLowerAbs" << armLowerAbs.x << armLowerAbs.y << DEG(armLowerAbs.theta);


    RobotPos armWristAbs;
    armWristAbs.theta = armLowerAbs.theta + RAD(_servoPos[ARM_LL_SERVO_LOWER_ARM]);

    armWristAbs.x = armLowerAbs.x + lenWrist * cos(armWristAbs.theta);
    armWristAbs.y = armLowerAbs.y + lenWrist * sin(armWristAbs.theta);

    pos.x = armWristAbs.x;
    pos.y = armWristAbs.y;

    _wristAngle = DEG(armWristAbs.theta);

    return pos;
}

void ArmLowLevelTest::checkObjects()
{
    // update X/Y/Z position of wrist
    Pos3D wristPosAbs = getPos();

    if (_catchedObject == nullptr)
    {
        tInfo(LOG) << "ArmHighLevelTest: checkObjects WristPosAbs" << wristPosAbs.x << wristPosAbs.y << wristPosAbs.z;

        _curDistance = 255.0;
        std::list<TableObject>::iterator it;
        for (it=_listObjects.begin(); it!=_listObjects.end(); ++it)
        {

            double dist = it->getDistance(wristPosAbs,_mode);

            if (dist < _curDistance && dist >= 0)
                _curDistance = dist;

            if (dist < 5 && dist >= 0 && it->isCatchable())
            {
                tInfo(LOG) << "ArmHighLevelTest: item can be catched !!" << dist;

                if (_vacuumValve == false && _vacuumPower > 0.0)
                {
                    it->catchObject();

                    _catchedObject = &(*it);
                }
            }
            //tInfo(LOG) << "ArmLowLevelTest: generate " << it->r << it->theta;
        }
    } else {
        if (_vacuumValve == true || _vacuumPower == 0.0)
        {
            _catchedObject->releaseObject(wristPosAbs,_mode);
            _catchedObject = nullptr;
        }
    }
}

// general
void ArmLowLevelTest::disable()
{
    enableZ(false);
    enableServo(ARM_LL_SERVO_UPPER_ARM,false);
    enableServo(ARM_LL_SERVO_LOWER_ARM,false);
    enableServo(ARM_LL_SERVO_WRIST,false);

    tInfo(LOG) << "ArmHighLevelTest: disable";

}

// Motor Z
void ArmLowLevelTest::enableZ(bool enable)
{
    tInfo(LOG) << "ArmHighLevelTest: enable Z" << enable;

    _zEnabled = enable;
}
void ArmLowLevelTest::setZ(double mmAbs)
{
    tInfo(LOG) << "ArmHighLevelTest: setZ" << mmAbs;

    _zPosition = mmAbs;
    // check function objects
    checkObjects();
}
double ArmLowLevelTest::getZ()
{
    tInfo(LOG) << "ArmHighLevelTest: getZ" << _zPosition;

    return _zPosition;
}
bool ArmLowLevelTest::waitZTargetOk(double timeoutMs)
{
    return true;
}
void ArmLowLevelTest::setZSpeed(double speed)
{

}
void ArmLowLevelTest::setZAcc(double acc)
{

}

// Servos
void ArmLowLevelTest::enableServo(enum ArmLowLevelLeg id, bool enable)
{
    tInfo(LOG) << "ArmHighLevelTest: enableServo" << id << enable;

    _servoEnable[id] = enable;
}
void ArmLowLevelTest::setServoPos(enum ArmLowLevelLeg id, double angleDegs)
{
    tInfo(LOG) << "ArmHighLevelTest: setServo" << id << angleDegs;

    _servoPos[id] = angleDegs;

    // check function objects
    checkObjects();
}
void ArmLowLevelTest::setServosPos(double angleDegs1, double angleDegs2, double angleDegs3)
{
    setServoPos(ARM_LL_SERVO_UPPER_ARM,angleDegs1);
    setServoPos(ARM_LL_SERVO_LOWER_ARM,angleDegs2);
    setServoPos(ARM_LL_SERVO_WRIST,angleDegs3);
}
double ArmLowLevelTest::getServoPos(enum ArmLowLevelLeg id)
{
    return _servoPos[id];
}
bool ArmLowLevelTest::waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs)
{
    return true;
}
bool ArmLowLevelTest::waitServosTargetOk(double timeoutMs)
{
    return true;
}

// Vacuum
void ArmLowLevelTest::setVacuumPower(float percentage)
{
    tInfo(LOG) << "ArmHighLevelTest: setVacuumPower" << percentage;

    _vacuumPower = percentage;

    // check function objects
    checkObjects();
}
void ArmLowLevelTest::setVacuumValve(bool enable)
{
    tInfo(LOG) << "ArmHighLevelTest: setVacuumValve" << enable;

    _vacuumValve = enable;

    // check function objects
    checkObjects();
}

// Distance
double ArmLowLevelTest::getDistance()
{
    tInfo(LOG) << "ArmHighLevelTest: getDistance" << _curDistance;

    return _curDistance+14.0;
}

bool ArmLowLevelTest::isDistanceCoherent()
{
    return true;
}
