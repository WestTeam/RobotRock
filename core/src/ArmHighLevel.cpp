// Copyright (c) 2019 All Rights Reserved WestBot

#include <cmath>

#include <QThread>
#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ArmHighLevel.hpp>


int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime)
{
  double a, dx, dy, d, h, rx, ry;
  double x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = x1 - x0;
  dy = y1 - y0;

  /* Determine the straight-line distance between the centers. */
  //d = sqrt((dy*dy) + (dx*dx));
  d = hypot(dx,dy); // Suggested by Keith Briggs

  /* Check for solvability. */
  if (d > (r0 + r1))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < fabs(r0 - r1))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  /* Determine the coordinates of point 2. */
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((r0*r0) - (a*a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h/d);
  ry = dx * (h/d);

  /* Determine the absolute intersection points. */
  *xi = x2 + rx;
  *xi_prime = x2 - rx;
  *yi = y2 + ry;
  *yi_prime = y2 - ry;

  return 1;
}


using namespace WestBot;
using namespace WestBot::RobotRock;

/* circle_circle_intersection() *
 * Determine the points where 2 circles in a common plane intersect.
 *
 * int circle_circle_intersection(
 *                                // center and radius of 1st circle
 *                                double x0, double y0, double r0,
 *                                // center and radius of 2nd circle
 *                                double x1, double y1, double r1,
 *                                // 1st intersection point
 *                                double *xi, double *yi,
 *                                // 2nd intersection point
 *                                double *xi_prime, double *yi_prime)
 *
 * This is a public domain work. 3/26/2005 Tim Voght
 *
 */
//#include <stdio.h>
//#include <math.h>


ArmHighLevel::ArmHighLevel()
    : _attached( false )
    //, _initOk( false )
    , _odometry( nullptr )
    , _armLL( nullptr )
{
    _armPos = { .x=0.0, .y=0.0, .theta = 0.0 };

    _storagePuckCount = 0;


/*
          ArmHighLevelMode _mode;

          Odometry::Ptr _odometry;
          ArmLowLevelBase::Ptr _armLL;

          RobotPos _armPos;
          RobotPos _storagePos[2];
          uint8_t _storagePuckCount[2];
*/
}

ArmHighLevel::~ArmHighLevel()
{

    tDebug( LOG ) << "ArmHighLevel destructor engaged";


    if (_attached)
    {
        disable();
    }

}


bool ArmHighLevel::init(
            const Odometry::Ptr& odometry,
            const ArmLowLevelBase::Ptr& armLL
        )
{
    bool ret = true;
    _attached = true;

    _odometry = odometry;
    _armLL = armLL;

    return ret;
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
    void confStorage(enum ArmHighLevelStorage id, double xMm,double yMm, double zMm);

    // move Arm to absolute position
    void moveArmAbs(double xMm, double yMm);

    // move Z (take into account the current H/V mode)
    void moveZ(double ZMm);

    void setMode(enum ArmHighLevelMode mode);

    // absolute value of detected object
    void getObjectPos(double &xMm, double &yMm, double &zMm);

    double getObjectDistance();

    uint8_t getPuckCount(enum ArmHighLevelStorage id);

    ///// ACTION /////
    bool actionSafePosition();
    bool actionGroundPuckCollection(double xMm, double yMm);
    bool actionDistributorPuckCollection(double xMm, double yMm);
    bool actionCheckGoldDoorOpen(double xMm, double yMm, double);
    bool actionGoldPuckCollection(double xMm, double yMm);
    bool actionPuckStore(enum ArmHighLevelStorage id);
    bool actionPuckUnstore(enum ArmHighLevelStorage id);

*/

void ArmHighLevel::enable()
{
    if (_attached)
    {
        _armLL->enableServo(ARM_LL_SERVO_UPPER_ARM,true);
        _armLL->enableServo(ARM_LL_SERVO_LOWER_ARM,true);
        _armLL->enableServo(ARM_LL_SERVO_WRIST,true);

        _armLL->enableZ(true);

        actionSafePosition();
    }
    tInfo( LOG ) <<  "ArmHighLevel Enabled";
}


void ArmHighLevel::disable()
{
    if (_attached)
    {
        _armLL->disable();
    }
    tInfo( LOG ) <<  "ArmHighLevel Disabled";
}

bool ArmHighLevel::isAttached() const
{
    return _attached;
}


void ArmHighLevel::confArmPos(double xMm,double yMm)
{
    _armPos.x = xMm;
    _armPos.y = yMm;
}

void ArmHighLevel::getArmPos(double &xMm,double &yMm)
{
    xMm = _armPos.x;
    yMm = _armPos.y;
}

void ArmHighLevel::confStorage(double xMm,double yMm, double zMm)
{
    _storagePos.x = xMm;
    _storagePos.y = yMm;
    _storagePos.z = zMm;
}

// move Arm to absolute position
bool ArmHighLevel::moveArmAbs(double xMm, double yMm)
{
    RobotPos rpos = _odometry->getPosition();
    // convert input data to relative ones to use moveArmRel function;
    double r = hypot(xMm-rpos.x,yMm-rpos.y);
    double theta = atan2(yMm-rpos.y,xMm-rpos.x);

    double thetaRel = theta - rpos.theta;

    double xRel = r*cos(thetaRel);
    double yRel = r*sin(thetaRel);

    return moveArmRel(xRel,yRel);
}

#define WRIST_AND_SUCTION_LENGTH 35.0

// move Arm to relative position from robot center
bool ArmHighLevel::moveArmRel(double xMm, double yMm)
{
/*
int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime)
*/

    tInfo( LOG ) <<  "ArmHighLevel: moveArmRel" << xMm << yMm;


    double x0,y0,r0;
    double x1,y1,r1;
    double xi,yi;
    double xi_prime,yi_prime;


    x0 = _armPos.x;
    y0 = _armPos.y;
    r0 = 70.0;

    x1 = xMm;
    y1 = yMm;
    r1 = 70.0;

    if (_mode == ARM_HL_MODE_VERTICAL)
        r1 += WRIST_AND_SUCTION_LENGTH;

    int ok = circle_circle_intersection(x0,y0,r0,
                               x1,y1,r1,
                               &xi,&yi,
                               &xi_prime,&yi_prime);

    tDebug(LOG) << "ArmHighLevel: circle_circle_intersection: pos " << xi << yi;
    tDebug(LOG) << "ArmHighLevel: circle_circle_intersection: pos prime " << xi_prime << yi_prime;


    if (ok == 1)
    {
        double theta1 = atan2(yi - y0,xi - x0);
        double theta2 = atan2(y1 - yi,x1 - xi)-theta1;

        tDebug(LOG) << "ArmHighLevel: moveArmRel atan2: " << y1 - yi << x1 - xi << theta2 << DEG(theta2);


        double theta1_prime = atan2(yi_prime - y0,xi_prime - x0);
        double theta2_prime = atan2(y1 - yi_prime,x1 - xi_prime)-theta1_prime;

        tDebug(LOG) << "ArmHighLevel: moveArmRel atan2 prime : " << y1 - yi_prime << x1 - xi_prime << theta2_prime << DEG(theta2_prime);


        if (theta1>M_PI)
            theta1-=2*M_PI;
        if (theta1<=-M_PI)
            theta1+=2*M_PI;

        if (theta2>M_PI)
            theta2-=2*M_PI;
        if (theta2<=-M_PI)
            theta2+=2*M_PI;


        if (theta1_prime>M_PI)
            theta1_prime-=2*M_PI;
        if (theta1_prime<=-M_PI)
            theta1_prime+=2*M_PI;

        if (theta2_prime>M_PI)
            theta2_prime-=2*M_PI;
        if (theta2_prime<=-M_PI)
            theta2_prime+=2*M_PI;


        tDebug(LOG) << "ArmHighLevel: moveArmRel angles: " << DEG(theta1) << DEG(theta2);
        tDebug(LOG) << "ArmHighLevel: moveArmRel angles primes: " << DEG(theta1_prime) << DEG(theta2_prime);


        double selected_theta1 = theta1;
        double selected_theta2 = theta2;

#define SERVO_UPPER_MAX_ANGLE_DEG 95.0
#define SERVO_LOWER_MAX_ANGLE_DEG 150.0


        // check if the angles are feasable
        if (!(std::abs(DEG(theta1)) <= SERVO_UPPER_MAX_ANGLE_DEG && std::abs(theta2) <= SERVO_LOWER_MAX_ANGLE_DEG))
        {
            selected_theta1 = theta1_prime;
            selected_theta2 = theta2_prime;

            tDebug(LOG) << "ArmHighLevel: circle_circle_intersection: solution 1 angles are too big " << DEG(theta1) << DEG(theta2);

            if (!(std::abs(DEG(theta1_prime)) <= SERVO_UPPER_MAX_ANGLE_DEG && std::abs(theta2_prime) <= SERVO_LOWER_MAX_ANGLE_DEG))
            {
                tWarning(LOG) << "ArmHighLevel: circle_circle_intersection: solution 1 & 2 angles are too big " << DEG(theta1_prime) << DEG(theta2_prime);
                return false;
            }
        }


        // we select the smallest theta
        // should we need to also keep the closest solution compared to the current one ? to
        // avoid doing too much movements
        if (0 && std::abs(theta2_prime) < std::abs(theta2))
        {
            selected_theta1 = theta1_prime;
            selected_theta2 = theta2_prime;
        }



        _armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,DEG(selected_theta1));
        _armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,DEG(selected_theta2));

        return _armLL->waitServosTargetOk(2000);
    } else {
        tWarning(LOG) << "ArmHighLevel: circle_circle_intersection: no solutions " << ok;
        return false;
    }

}

// move Z (take into account the current H/V mode)
void ArmHighLevel::moveZ(double ZMm)
{
    double offset = 0.0;
    if (_mode == ARM_HL_MODE_HORIZONTAL)
        offset = WRIST_AND_SUCTION_LENGTH;

    _armLL->setZ(ZMm + offset);
    _armLL->waitZTargetOk(2000);
}

void ArmHighLevel::setMode(enum ArmHighLevelMode mode)
{    _mode = mode;
    if (_mode == ARM_HL_MODE_HORIZONTAL)
        _armLL->setServoPos(ARM_LL_SERVO_WRIST,-90.0);
    else
        _armLL->setServoPos(ARM_LL_SERVO_WRIST,0.0);

    _armLL->waitServoTargetOk(ARM_LL_SERVO_WRIST,1000);
}

void ArmHighLevel::setVacuum(bool enable)
{
    _armLL->setVacuumPower(enable?100.0:0.0);
    _armLL->setVacuumValve(!enable);
}

// absolute value of detected object
void ArmHighLevel::getObjectPos(double &xMm, double &yMm, double &zMm)
{
    double angle1, angle2, angle3;

    angle1 = _armLL->getServoPos(ARM_LL_SERVO_UPPER_ARM);
    angle2 = _armLL->getServoPos(ARM_LL_SERVO_LOWER_ARM);
    angle3 = _armLL->getServoPos(ARM_LL_SERVO_WRIST);

    double armX0,armY0;

    RobotPos rpos = _odometry->getPosition();

    armX0 = rpos.x + _armPos.x*cos(rpos.theta);
    armY0 = rpos.y + _armPos.y*sin(rpos.theta);

    double armX1,armY1;

    armX1 = armX0 + 70.0*cos(rpos.theta+RAD(angle1));
    armY1 = armY0 + 70.0*sin(rpos.theta+RAD(angle1));

    double armX2,armY2;

    armX2 = armX1 + 70.0*cos(rpos.theta+RAD(angle1+angle2));
    armY2 = armY1 + 70.0*sin(rpos.theta+RAD(angle1+angle2));
    double curZ = _armLL->getZ();

    double armX3,armY3;
#define WRIST_LENGTH 24.6
    double len3 = WRIST_LENGTH + _armLL->getDistance();

    if (_mode == ARM_HL_MODE_VERTICAL)
    {
        armX3 = armX2 + len3*cos(rpos.theta+RAD(angle1+angle2));
        armY3 = armY2 + len3*sin(rpos.theta+RAD(angle1+angle2));
    }
    else
    {
        armX3 = armX2;
        armY3 = armY2;

        curZ -= len3;
    }

    xMm = armX3;
    yMm = armY3;
    zMm = curZ;
}

double ArmHighLevel::getObjectDistance()
{
#define SUCTION_CUP_LENGTH 10.0

    double dist = _armLL->getDistance();
    double distObj = dist;
    distObj -= SUCTION_CUP_LENGTH;
    if (distObj < 0)
        distObj = 0.0;

    tDebug(LOG) << "ArmHighLevel::getObjectDistance" << dist << distObj;
    return distObj;
}

uint8_t ArmHighLevel::getPuckCount()
{
    return _storagePuckCount;
}

///// ACTION /////
bool ArmHighLevel::actionSafePosition()
{
    setVacuum(false);

    setMode(ARM_HL_MODE_HORIZONTAL);

    // TO BE DEFINED
#define SAFE_OUT_ANGLE_1 0.0
#define SAFE_OUT_ANGLE_2 0.0

#define SAFE_IN_ANGLE_1 0.0
#define SAFE_IN_ANGLE_2 0.0

#define SAFE_Z 200.0


    _armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,SAFE_OUT_ANGLE_1);
    _armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,SAFE_OUT_ANGLE_2);

    _armLL->waitServosTargetOk(2000);

    moveZ(SAFE_Z);

    _armLL->waitZTargetOk(2000);

    _armLL->setServoPos(ARM_LL_SERVO_UPPER_ARM,SAFE_IN_ANGLE_1);
    _armLL->setServoPos(ARM_LL_SERVO_LOWER_ARM,SAFE_IN_ANGLE_2);

    _armLL->waitServosTargetOk(2000);


    return _armLL->waitServosTargetOk(1) && _armLL->waitZTargetOk(1);
}

bool ArmHighLevel::actionGroundPuckCollection(double xMm, double yMm)
{
    setVacuum(true);

    setMode(ARM_HL_MODE_HORIZONTAL);
    moveArmAbs(xMm,yMm);
#define PUCK_WIDTH 25.0

    moveZ(PUCK_WIDTH+PUCK_WIDTH*2);
    QThread::msleep(100);
    double dist = getObjectDistance();
    tDebug(LOG) << dist << _armLL->getDistance() << _armLL->isDistanceCoherent();
    if (dist >= PUCK_WIDTH*2.5)
    {
        tWarning(LOG) << "ArmHighLevel: actionGroundPuckCollection: Distance too big (no puck?)" << dist;
        setVacuum(false);
        return false;
    }

    moveZ(PUCK_WIDTH+3.5);//+PUCK_WIDTH*2-dist-3.0);

    //QThread::msleep(200);

    //moveZ(PUCK_WIDTH);

    moveZ(PUCK_WIDTH+50);

    dist = getObjectDistance();
    if (dist >= 10)
    {
        tWarning(LOG) << "ArmHighLevel: actionGroundPuckCollection: Did not succeed to catch " << dist;
        setVacuum(false);
        return false;
    }

    return true;
}

bool ArmHighLevel::actionDistributorPuckCollection(double xMm, double yMm)
{
    setVacuum(true);

    setMode(ARM_HL_MODE_VERTICAL);
#define DISTRI_HEIGHT 100.0
#define PUCK_DIAMETER 76.0
    moveZ(DISTRI_HEIGHT+PUCK_DIAMETER/2);

    // only work because orientation of the robot is the same for all distributors
    moveArmAbs(xMm-SUCTION_CUP_LENGTH,yMm);

    double dist = getObjectDistance();
    if (dist >= PUCK_WIDTH)
    {
        tWarning(LOG) << "ArmHighLevel: actionDistributorPuckCollection: Distance too big (no puck?)" << dist;
        setVacuum(false);
        return false;
    }
    moveArmAbs(xMm,yMm);

    dist = getObjectDistance();
    if (dist >= 10)
    {
        tWarning(LOG) << "ArmHighLevel: actionGroundPuckCollection: Do no see puck " << dist;
        setVacuum(false);
        return false;
    }

    // move on puck diamter higher
    moveZ(DISTRI_HEIGHT+PUCK_DIAMETER+PUCK_DIAMETER/2);

    dist = getObjectDistance();
    if (dist >= 10)
    {
        tWarning(LOG) << "ArmHighLevel: actionGroundPuckCollection: Puck lost during Z move " << dist;
        setVacuum(false);
        return false;
    }

    return true;
}

bool ArmHighLevel::actionCheckGoldDoorOpen(double xMm, double yMm)
{
    setMode(ARM_HL_MODE_VERTICAL);

#define GOLD_CHECKPOINT_Z (165.0+PUCK_DIAMETER/2)

    moveZ(GOLD_CHECKPOINT_Z);

    moveArmAbs(xMm+20.0,yMm);

    double dist = getObjectDistance();

    if (dist > 30.0)
    {
        tWarning(LOG) << "ArmHighLevel: actionCheckGoldDoorOpen: Gold door not open " << dist;
        return false;
    }

    return true;
}

bool ArmHighLevel::actionGoldPuckCollection(double xMm, double yMm)
{
    setVacuum(true);
    setMode(ARM_HL_MODE_VERTICAL);

    moveZ(GOLD_CHECKPOINT_Z);

    moveArmAbs(xMm+30.0,yMm);
    double dist = getObjectDistance();

    if (dist > 30.0+PUCK_DIAMETER)
    {
        tWarning(LOG) << "ArmHighLevel: actionGoldPuckCollection: Gold puck not found " << dist;
        return false;
    }

    moveArmAbs(xMm,yMm);

    dist = getObjectDistance();

    if (dist > 5.0)
    {
        tWarning(LOG) << "ArmHighLevel: actionGoldPuckCollection: Gold puck not grabbed " << dist;
        return false;
    }

    // next the user needs to go backward in order to remove the gold puck from its location

    return true;
}

bool ArmHighLevel::actionPuckStore()
{
    setMode(ARM_HL_MODE_HORIZONTAL);

#define STORE_Z 200.0

    moveZ(STORE_Z);

    QThread::msleep(500);

    moveArmRel(_storagePos.x,_storagePos.y);

    uint8_t count = getPuckCount();

    double targetZ = _storagePos.z + count*PUCK_WIDTH + PUCK_WIDTH + 10.0;

    moveZ(targetZ);

    setVacuum(false);

    QThread::msleep(500);


    targetZ = _storagePos.z + count*PUCK_WIDTH + PUCK_WIDTH + PUCK_WIDTH;

    moveZ(targetZ);

    QThread::msleep(200);

    double dist = getObjectDistance();

    if (dist > (PUCK_WIDTH+10.0) || dist < 15.0 )
    {
        tWarning(LOG) << "ArmHighLevel: actionPuckStore: Puck store action issue " << dist;
        return false;
    }

    _storagePuckCount++;

    return true;

}

bool ArmHighLevel::actionPuckUnstore()
{
    setMode(ARM_HL_MODE_HORIZONTAL);

    moveZ(STORE_Z);
    moveArmRel(_storagePos.x,_storagePos.y);

    uint8_t count = getPuckCount();


    double targetZ = _storagePos.z + count*PUCK_WIDTH  + 30.0;


    tDebug(LOG) <<  "ArmHighLevel: actionPuckUnstore: getPuckCount / targetZ" << count << targetZ;


    double dist = getObjectDistance();

    if (dist < (STORE_Z-targetZ))
    {
        tWarning(LOG) << "ArmHighLevel: actionPuckUnstore: invalid data (number of puck store?)" << dist << count;
        return false;
    }

    moveZ(targetZ);

    setVacuum(true);

    QThread::msleep(500);

    dist = getObjectDistance();

    targetZ -= dist;

    tDebug(LOG) <<  "ArmHighLevel: actionPuckUnstore: Before catch: dist / targetZ" << dist << targetZ;

    if (targetZ < (_storagePos.z + (PUCK_WIDTH-8.0)))
    {
        targetZ = _storagePos.z + (PUCK_WIDTH);
        tWarning(LOG) << "ArmHighLevel: actionPuckUnstore: no puck found while we expected" << count << "dist:" << dist;
    }

    moveZ(targetZ);

    dist = getObjectDistance();

    if (dist > 5.0)
    {
        tWarning(LOG) << "ArmHighLevel: actionPuckUnstore: puck not found" << dist;
        return false;
    }

    targetZ += PUCK_WIDTH;

    moveZ(targetZ);


    dist = getObjectDistance();

    if (dist > 5.0)
    {
        tWarning(LOG) << "ArmHighLevel: actionPuckUnstore: puck not catched(retry needed?)" << dist;
        return false;
    }

    _storagePuckCount--;

    return true;
}


bool ArmHighLevel::actionPuckRelease(double xMm, double yMm, double zMm)
{
    bool ret = true;

    double initDist = getObjectDistance();
    double dist1, dist2, dist3;

#define DISTANCE_ERROR (10.0)

    if (initDist > DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmHighLevel::actionPuckRelease Init getObjectDistance too big (puck not locked?)" << initDist;
    }

    moveArmAbs(xMm,yMm);

    dist1 = getObjectDistance();

    if (dist1 > DISTANCE_ERROR && initDist <= DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmHighLevel::actionPuckRelease After Z Move getObjectDistance too big (puck lost?)" << dist1 << initDist;
    }

    moveZ(zMm);

    dist2 = getObjectDistance();

    if (dist2 > DISTANCE_ERROR && dist1 <= DISTANCE_ERROR && initDist <= DISTANCE_ERROR)
    {
        ret = false;
        tWarning(LOG) << " ArmHighLevel::actionPuckRelease After Arm Move getObjectDistance too big (puck lost?)" << dist2 << dist1 << initDist;
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
        tWarning(LOG) << " ArmHighLevel::actionPuckRelease After vaccum off Move getObjectDistance too close (puck still locked?)" << dist3;
    }

    return ret;
}

