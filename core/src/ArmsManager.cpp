// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>

#include <QThread>

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

    _score = 0;
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

bool getCatchPosition(PuckPos* left1, PuckPos* left2, PuckPos* right1, PuckPos* right2, RobotPos &pos);
// get a list of pucks for each arm and store each of them (if the last one cannot be stored (full), we keep it outside)
bool getPucksAndStore(PuckPos* left1, PuckPos* left2, PuckPos* right1, PuckPos* right2);
void getPucksAndStoreSingle(bool isRight, PuckPos* puck1, PuckPos* puck2, bool *ret);



bool ArmsManager::getCatchPosition(PuckPos* left1, PuckPos* left2, PuckPos* right1, PuckPos* right2, RobotPos &pos)
{
    bool ret = false;

    double arml_pos_x,arml_pos_y;
    double armr_pos_x,armr_pos_y;

    double arml_abs_x,arml_abs_y;
    double armr_abs_x,armr_abs_y;

    _arm[0]->getArmPos(arml_pos_x,arml_pos_y);
    _arm[1]->getArmPos(armr_pos_x,armr_pos_y);

    RobotPos rpos = _odometry->getPosition();

    arml_abs_x = rpos.x+arml_pos_x*cos(rpos.theta);
    arml_abs_y = rpos.y+arml_pos_y*sin(rpos.theta);

    armr_abs_x = rpos.x+armr_pos_x*cos(rpos.theta);
    armr_abs_y = rpos.y+armr_pos_y*sin(rpos.theta);

    double x,y;

    int leftCount = (left1!=nullptr) + (left2!=nullptr);
    int rightCount = (right1!=nullptr) + (right2!=nullptr);


    if (leftCount == 1 && rightCount == 1)
    {
        if (left1->isOnGround == false)
        {

            double diff_arm_puck_y = left1->y-arml_pos_y;

            y = left1->y+(left1->y-left1->y)/2;
            x = left1->x
              - (ARM_LOWER_LENGTH+WRIST_AND_SUCTION_LENGTH)
              - sqrt(ARM_UPPER_LENGTH*ARM_UPPER_LENGTH+diff_arm_puck_y*diff_arm_puck_y)
              - arml_pos_x;

            pos.theta = left1->theta-M_PI;
            pos.x = x;
            pos.y = y;

            ret = true;
        } else {
            // not yet handled
        }
    }
    if ((leftCount == 1 && rightCount == 0) || (leftCount == 0 && rightCount == 1))
    {
        double inv=1.0;
        PuckPos *puck = right1;

        if (leftCount == 1 && rightCount == 0)
        {
            inv = -1.0;
            puck = left1;
        }

        if (puck->isOnGround == true)
        {
            double d,theta;

            d = hypot(puck->x-rpos.x,puck->y-rpos.y);
            theta = atan2(puck->y-rpos.y,puck->x-rpos.x);

            //theta-=rpos.theta;
            d -= arml_abs_x-70.0;


            x = rpos.x+d*cos(theta);
            y = rpos.y+d*sin(theta);

            rpos.x = x;
            rpos.y = y;
            rpos.theta = theta;

            ret = true;
        } else {

            x = puck->x + (WRIST_AND_SUCTION_LENGTH + ARM_UPPER_LENGTH + ARM_LOWER_LENGTH + arml_pos_x)*cos(puck->theta);
            y = puck->y + arml_pos_y*inv*cos(puck->theta)*-1.0;

            rpos.x = x;
            rpos.y = y;
            rpos.theta = puck->theta-M_PI;

            ret = true;
        }


    }

    return ret;
}


bool ArmsManager::getPucksAndStore(PuckPos* left1, PuckPos* left2, PuckPos* right1, PuckPos* right2)
{
    bool rleft,rright;

    std::thread tl(&ArmsManager::getPucksAndStoreSingle,this,false,left1,left2,&rleft);
    std::thread tr(&ArmsManager::getPucksAndStoreSingle,this,true,right1,right2,&rright);

    tl.join();
    tr.join();

    return rleft && rright;
}

void ArmsManager::getPucksAndStoreSingle(bool isRight, PuckPos* puck1, PuckPos* puck2, bool *ret)
{
    *ret = true;
    bool actionOk;



    QList<PuckPos*> list;
    if (puck1)
        list << puck1;
    if (puck2)
        list << puck2;

    for (QList<PuckPos*>::iterator it = list.begin(); it != list.end(); ++it){
        if ((*it)->isOnGround)
        {
            actionOk = _arm[isRight]->actionGroundPuckCollection((*it)->x,(*it)->y);

            *ret &= actionOk;
            if (actionOk)
            {
                actionOk = _arm[isRight]->actionPuckStore();

                if (actionOk)
                    _pucksStored[isRight] << (*it);
                *ret &= actionOk;
            }
        }
    }
}


bool ArmsManager::getPucks(PuckPos *left, PuckPos *right)
{
    bool rleft,rright;

    std::thread tl(&ArmsManager::getPuck,this,false,left,&rleft);
    std::thread tr(&ArmsManager::getPuck,this,true,right,&rright);

    tl.join();
    tr.join();

    return rleft && rright;
}

void ArmsManager::getPuck(bool isRight, PuckPos *puck, bool *ret)
{
    if (puck != nullptr)
    {
        if (puck->isOnGround)
        {
            bool actionOk = _arm[isRight]->actionGroundPuckCollection(puck->x,puck->y);

            if (actionOk)
                _pucksAttached[isRight] = puck;

            double safeX = 270.0;
            double safeY = 50.0;

            if (isRight)
                safeY*=-1.0;

            _arm[isRight]->moveArmRel(safeX,safeY);

            *ret = actionOk;
        }
    } else {
        *ret = true;
    }
}


void ArmsManager::getReleaseAcceleratorPosition(RobotPos &pos)
{

}

bool ArmsManager::releasePucksAcceletator()
{
    bool rleft,rright;

    std::thread tl(&ArmsManager::releasePucksAcceletatorSingle,this,false,&rleft);
    std::thread tr(&ArmsManager::releasePucksAcceletatorSingle,this,true,&rright);

    return rleft && rright;

}


void ArmsManager::releasePucksAcceletatorSingle(bool isRight, bool *ret)
{
    *ret = true;
    bool actionOk;

    if (_pucksAttached[isRight])
    {
        _arm[isRight]->moveZ(180.0);
        _arm[isRight]->setMode(ARM_HL_MODE_VERTICAL);
        _arm[isRight]->moveArmRel(340.0,115.0);
        _arm[isRight]->setVacuum(false);
        _score+=10;
    }

    _pucksAttached[isRight] = nullptr;
/*
    for (QList<PuckPos*>::iterator it = list.begin(); it != list.end(); ++it){
        if ((*it)->isOnGround)
        {
            actionOk = _arm[isRight]->actionGroundPuckCollection((*it)->x,(*it)->y);

            *ret &= actionOk;
            if (actionOk)
            {
                actionOk = _arm[isRight]->actionPuckStore();

                if (actionOk)
                    _pucksStored[isRight] << (*it);
                *ret &= actionOk;
            }
        }
    }
    */
}


void ArmsManager::getReleaseScalePosition(RobotPos &pos)
{

}

bool ArmsManager::releasePucksScale()
{


    _pucksAttached[false] = nullptr;
    _pucksAttached[true] = nullptr;
}

void ArmsManager::getReleaseGroundPosition(RobotPos &pos)
{

}

bool ArmsManager::releasePucksGround()
{

    _pucksAttached[false] = nullptr;
    _pucksAttached[true] = nullptr;
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

