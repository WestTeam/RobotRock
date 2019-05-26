// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>

#include <QThread>
#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmsManager.hpp>

#include <WestBot/RobotRock/ArmsManager.hpp>


using namespace WestBot;
using namespace WestBot::RobotRock;


ArmsManager::ArmsManager()
    : _attached( false )
    , _score(0)
    , _isPurple(0)
    , _odometry( nullptr )
{

    _arm[0] = nullptr;
    _arm[1] = nullptr;

}

ArmsManager::~ArmsManager()
{

    tDebug( LOG ) << "ArmsManager destructor engaged";


    if (_attached)
    {
        disable();
    }

}


bool ArmsManager::init(
        const Odometry::Ptr& odometry,
        const ArmHighLevel::Ptr& armLeft,
        const ArmHighLevel::Ptr& armRight

        )
{
    bool ret = true;
    _attached = true;

    _odometry = odometry;
    _arm[0] = armLeft;
    _arm[1] = armRight;

    return ret;
}



void ArmsManager::setColor(bool isPurple)
{
    _isPurple = isPurple;
    _score = 0;
}

int ArmsManager::getScore()
{
    return _score;
}
/*

    bool init(
         const Odometry::Ptr& odometry, const ArmLowLevelBase::Ptr& armLL
    );

    bool isAttached() const;

    // general
    void disable();
    void enable();

    // position of the ARM axe compared to robot center
    void confArmPos(double xMm,double yMm);
    void confStorage(enum ArmsManagerStorage id, double xMm,double yMm, double zMm);

    // move Arm to absolute position
    void moveArmAbs(double xMm, double yMm);

    // move Z (take into account the current H/V mode)
    void moveZ(double ZMm);

    void setMode(enum ArmsManagerMode mode);

    // absolute value of detected object
    void getObjectPos(double &xMm, double &yMm, double &zMm);

    double getObjectDistance();

    uint8_t getPuckCount(enum ArmsManagerStorage id);

    ///// ACTION /////
    bool actionSafePosition();
    bool actionGroundPuckCollection(double xMm, double yMm);
    bool actionDistributorPuckCollection(double xMm, double yMm);
    bool actionCheckGoldDoorOpen(double xMm, double yMm, double);
    bool actionGoldPuckCollection(double xMm, double yMm);
    bool actionPuckStore(enum ArmsManagerStorage id);
    bool actionPuckUnstore(enum ArmsManagerStorage id);

*/

void ArmsManager::enable()
{
    if (_attached)
    {
        _arm[0]->enable();
        _arm[1]->enable();

    }
    tInfo( LOG ) <<  "ArmsManager Enabled";
}


void ArmsManager::disable()
{
    if (_attached)
    {
        _arm[0]->disable();
        _arm[1]->disable();
    }
    tInfo( LOG ) <<  "ArmsManager Disabled";
}

bool ArmsManager::isAttached() const
{
    return _attached;
}


/*
 *
    // get ideal position
    bool getCatchPosition(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight, RobotPos &pos);
    // get a list of pucks for each arm and store each of them (if the last one cannot be stored (full), we keep it outside)
    bool getPucksAndStore(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight);

    // if null, no one to take
    bool getPucks(PuckPos *left, PuckPos *right);

    void getReleaseAcceleratorPosition(RobotPos &pos);

    // release all stored pucks into the accelerator
    bool releasePucksAcceletator();

    void getReleaseScalePosition(RobotPos &pos);

    // release all stored pucks into the scale
    bool releasePucksScale();

    void getReleaseGroundPosition(RobotPos &pos);
    bool releasePucksGround();


    */

bool getCatchPosition(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight, RobotPos &pos)
{

}


bool getPucksAndStore(std::list<PuckPos> &listLeft, std::list<PuckPos> &listRight)
{

}

bool getPucks(PuckPos *left, PuckPos *right)
{

}

void getReleaseAcceleratorPosition(RobotPos &pos)
{

}

bool releasePucksAcceletator()
{

}

void getReleaseScalePosition(RobotPos &pos)
{

}

bool releasePucksScale()
{


}

void getReleaseGroundPosition(RobotPos &pos)
{

}

bool releasePucksGround()
{

}
/*
bool ArmsManager::actionPuckRelease(double xMm, double yMm, double zMm)
{
    bool ret = true;

    double initDist = getObjectDistance();
    double dist1, dist2, dist3;

#define DISTANCE_ERROR (10.0)

    if (initDist > DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmsManager::actionPuckRelease Init getObjectDistance too big (puck not locked?)" << initDist;
    }

    moveZ(zMm);

    dist1 = getObjectDistance();

    if (dist1 > DISTANCE_ERROR && initDist <= DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmsManager::actionPuckRelease After Z Move getObjectDistance too big (puck lost?)" << dist1 << initDist;
    }

    moveArmAbs(xMm,yMm);

    dist2 = getObjectDistance();

    if (dist2 > DISTANCE_ERROR && dist1 <= DISTANCE_ERROR && initDist <= DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmsManager::actionPuckRelease After Arm Move getObjectDistance too big (puck lost?)" << dist2 << dist1 << initDist;
    }

    setVacuum(false);

    // if we are in HORIZONTAL mode, we need to move to a higher level to check distance
    if (_mode == ARM_HL_MODE_HORIZONTAL)
        moveZ(zMm+PUCK_WIDTH);
    else
        moveZ(zMm+PUCK_DIAMETER);


    dist3 = getObjectDistance();

    if (dist3 <= DISTANCE_ERROR)
    {
        tWarning(LOG) << " ArmsManager::actionPuckRelease After vaccum off Move getObjectDistance too close (puck still locked?)" << dist3;
    }

    return ret;
}*/

