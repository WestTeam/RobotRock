#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/ArmLowLevel.hpp>
#include <WestBot/RobotRock/ArmHighLevel.hpp>


#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testArmLowLevel" )
}

/*
typedef struct
{
    double x; // mm
    double y; // mm
    double r;
    double theta;
} LidarPoint;

bool lidarPointCompare (const LidarPoint& first, const LidarPoint& second)
{
  return (first.theta < second.theta);
}
*/

/*

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
};

*/



class TableObject
{
public:
    TableObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode)
    {
        _resetPos = pos;
        _resetIsCatchable = isCatchable;
        _resetMode = mode;

        _pos = pos;
        _mode = mode;
        _isCatchable = isCatchable;
    }

    void reset() {
        _isCatched = false;

        _pos = _resetPos;
        _mode = _resetMode;
        _isCatchable = _resetIsCatchable;
    }

    bool isCatchable() {
        return _isCatchable;
    }

    bool isCatched() {
        return _isCatched;
    }

    double getDistance(Pos3D wristPos, enum ArmHighLevelMode mode)
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

    void catchObject()
    {
        if (_isCatchable)
        {
            _isCatched = true;
            tInfo(LOG) << "TableObject: catched!" << _pos.x << _pos.y << _pos.z;
        }
    }
    void releaseObject(Pos3D wristPos, enum ArmHighLevelMode mode)
    {
        _pos = wristPos;
        _mode = mode;
        _isCatched = false;

        tInfo(LOG) << "TableObject: released!" << _pos.x << _pos.y << _pos.z << _mode;

    }

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

    ArmLowLevelTest(const Odometry::Ptr odometry, RobotPos armPos)
    {
        _mode = ARM_HL_MODE_HORIZONTAL;
        _armPos = armPos;
        _curDistance = 255.0;
        _odometry = odometry;
        _catchedObject = nullptr;
        tInfo(LOG) << "ArmLowLevelTest: Constructor";
    }

    ~ArmLowLevelTest()
    {
        tInfo(LOG) << "ArmLowLevelTest: Destructor";

    }

    void resetObjects()
    {
        _catchedObject = nullptr;

        std::list<TableObject>::iterator it;
        for (it=_listObjects.begin(); it!=_listObjects.end(); ++it)
        {
            it->reset();
        }
    }

    void addObject(TableObject object)
    {
        _listObjects.push_back(object);
    }

    void addObject(Pos3D pos, bool isCatchable, enum ArmHighLevelMode mode)
    {
        _listObjects.push_back(TableObject(pos,isCatchable,mode));
    }

    Pos3D getPos()
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
//        tInfo(LOG) << "LidarTest: angleCenterLidar " << angleCenterLidar;
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

    void checkObjects()
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
    void disable()
    {
        enableZ(false);
        enableServo(ARM_LL_SERVO_UPPER_ARM,false);
        enableServo(ARM_LL_SERVO_LOWER_ARM,false);
        enableServo(ARM_LL_SERVO_WRIST,false);

        tInfo(LOG) << "ArmHighLevelTest: disable";

    }

    // Motor Z
    void enableZ(bool enable)
    {
        tInfo(LOG) << "ArmHighLevelTest: enable Z" << enable;

        _zEnabled = enable;
    }
    void setZ(double mmAbs)
    {
        tInfo(LOG) << "ArmHighLevelTest: setZ" << mmAbs;

        _zPosition = mmAbs;
        // check function objects
        checkObjects();
    }
    double getZ()
    {
        tInfo(LOG) << "ArmHighLevelTest: getZ" << _zPosition;

        return _zPosition;
    }
    bool waitZTargetOk(double timeoutMs = 0)
    {
        return true;
    }
    void setZSpeed(double speed)
    {

    }
    void setZAcc(double acc)
    {

    }

    // Servos
    void enableServo(enum ArmLowLevelLeg id, bool enable)
    {
        tInfo(LOG) << "ArmHighLevelTest: enableServo" << id << enable;

        _servoEnable[id] = enable;
    }
    void setServoPos(enum ArmLowLevelLeg id, double angleDegs)
    {
        tInfo(LOG) << "ArmHighLevelTest: setServo" << id << angleDegs;

        _servoPos[id] = angleDegs;

        // check function objects
        checkObjects();
    }
    void setServosPos(double angleDegs1, double angleDegs2, double angleDegs3)
    {
        setServoPos(ARM_LL_SERVO_UPPER_ARM,angleDegs1);
        setServoPos(ARM_LL_SERVO_LOWER_ARM,angleDegs2);
        setServoPos(ARM_LL_SERVO_WRIST,angleDegs3);
    }
    double getServoPos(enum ArmLowLevelLeg id)
    {
        return _servoPos[id];
    }
    bool waitServoTargetOk(enum ArmLowLevelLeg id, double timeoutMs)
    {
        return true;
    }
    bool waitServosTargetOk(double timeoutMs)
    {
        return true;
    }

    // Vacuum
    void setVacuumPower(float percentage)
    {
        tInfo(LOG) << "ArmHighLevelTest: setVacuumPower" << percentage;

        _vacuumPower = percentage;

        // check function objects
        checkObjects();
    }
    void setVacuumValve(bool enable)
    {
        tInfo(LOG) << "ArmHighLevelTest: setVacuumValve" << enable;

        _vacuumValve = enable;

        // check function objects
        checkObjects();
    }

    // Distance
    double getDistance()
    {
        tInfo(LOG) << "ArmHighLevelTest: getDistance" << _curDistance;

        return _curDistance+14.0;
    }

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
/*
    std::default_random_engine re_error;
    std::default_random_engine re_missing;

    struct {
        bool dir;
        double	ax;
        double	ay;
        double	bx;
        double	by;
    } _tableBorder[BORDER_COUNT_MAX];

    int _tableBorderNb = 0;

    std::list<LidarPoint> _listPoint;
*/
};


void log_management(int argc, char *argv[]){
    QCoreApplication app(argc,argv);

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif

    app.exec();
}

int main( int argc, char *argv[] )
{
    std::thread thread_logs(log_management,argc,argv);


    Hal::Ptr hal = std::make_shared< Hal >();


    RobotPos armPos;

    armPos.x = 0.0;
    armPos.y = 0.0;

    Odometry::Ptr odometryPtr = std::make_shared< Odometry >( hal );

    odometryPtr->setPosition({.x=0.0,.y=0.0,.theta=RAD(0.0)});

    ArmLowLevelTest::Ptr armLLPtr = std::make_shared< ArmLowLevelTest >(odometryPtr,armPos);

    TableObject puck1({.x=100.0,.y=1.0,.z=200.0},true,ARM_HL_MODE_VERTICAL);
    TableObject puck2({.x=50.0,.y=50.0,.z=25.0},true,ARM_HL_MODE_HORIZONTAL);
    TableObject puck3({.x=25.0,.y=25.0,.z=25.0},true,ARM_HL_MODE_HORIZONTAL);

    TableObject goldenDoor({.x=-50.0,.y=-20.0,.z=165.0+76.0/2},false,ARM_HL_MODE_VERTICAL);
    TableObject goldenPuck({.x=-50.0,.y=-50.0,.z=165.0+76.0/2},true,ARM_HL_MODE_VERTICAL);

    TableObject distriPuck({.x=60.0,.y=60.0,.z=100.0+76.0/2},true,ARM_HL_MODE_VERTICAL);


    armLLPtr->addObject(puck1);
    armLLPtr->addObject(puck2);
    armLLPtr->addObject(puck3);
    armLLPtr->addObject(goldenDoor);
    armLLPtr->addObject(goldenPuck);
    armLLPtr->addObject(distriPuck);


    ///// GENERAL CONFIG /////

    unsigned int tId;
    unsigned int i;

    // TEST 1: Grab and release items
    {
        tId = 1;

        armLLPtr->resetObjects();

        ArmHighLevel armHL;

        armHL.init(odometryPtr,armLLPtr);

        armHL.enable();

        armHL.confArmPos(0.0,0.0);

        armHL.setMode(ARM_HL_MODE_VERTICAL);

        armHL.moveZ(200.0);

        armHL.setVacuum(true);

        armHL.moveArmAbs(100.0,0.0);


        //armHL.setMode(ARM_HL_MODE_HORIZONTAL);

        armHL.setMode(ARM_HL_MODE_HORIZONTAL);

        armHL.moveArmAbs(100.0,50.0);

        armHL.moveZ(25.0);

        armHL.setVacuum(false);

        armHL.moveZ(50.0);

        armHL.setVacuum(true);

        armHL.moveZ(25.0);

        armHL.moveZ(200.0);

        armHL.setMode(ARM_HL_MODE_VERTICAL);

        armHL.moveZ(200.0);


        armHL.moveArmAbs(100.0,0.0);

        armHL.setVacuum(false);

    } // END TEST 1


    // TEST 2: Grab and release items
    {
        tId = 2;

        tInfo(LOG) << "ArmHighLevel: TEST" << tId;

        armLLPtr->resetObjects();

        ArmHighLevel armHL;

        armHL.init(odometryPtr,armLLPtr);

        armHL.enable();

        armHL.confArmPos(0.0,0.0);

        armHL.confStorage(ARM_HL_STORAGE_LEFT,0,25,25);

        bool ret;

        ret = armHL.actionGroundPuckCollection(50.0,50.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionGroundPuckCollection 1 failed";
        }

        ret = armHL.actionPuckStore(ARM_HL_STORAGE_LEFT);


        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckStore 1 ARM_HL_STORAGE_LEFT failed";
        }


        ret = armHL.actionGroundPuckCollection(25.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionGroundPuckCollection 2 failed";
        }

        ret = armHL.actionPuckStore(ARM_HL_STORAGE_LEFT);


        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckStore 2 ARM_HL_STORAGE_LEFT failed";
        }

        if (armHL.getPuckCount(ARM_HL_STORAGE_LEFT) != 2)
        {
            tFatal(LOG) << "Test" << tId << "getPuckCount puck count issue" << armHL.getPuckCount(ARM_HL_STORAGE_LEFT);
        }

        ret = armHL.actionPuckUnstore(ARM_HL_STORAGE_LEFT);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckUnstore 1 ARM_HL_STORAGE_LEFT failed";
        }

        ret = armHL.actionPuckRelease(50.0,50.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckRelease 1 failed";
        }



        ret = armHL.actionPuckUnstore(ARM_HL_STORAGE_LEFT);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckUnstore 2 ARM_HL_STORAGE_LEFT failed";
        }

        ret = armHL.actionPuckRelease(25.0,25.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckRelease 2 failed";
        }

        if (armHL.getPuckCount(ARM_HL_STORAGE_LEFT) != 0)
        {
            tFatal(LOG) << "Test" << tId << "getPuckCount released puck count issue" << armHL.getPuckCount(ARM_HL_STORAGE_LEFT);
        }

        ret = armHL.actionCheckGoldDoorOpen(-50.0,-20.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionCheckGoldDoorOpen  check fail";
        }

        ret = armHL.actionGoldPuckCollection(-50.0,-50.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionGoldPuckCollection  check fail";
        }

        ret = armHL.actionPuckRelease(-50,20.0,76.0/2);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckRelease Golden fail";
        }

        ret = armHL.actionDistributorPuckCollection(60,60.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionDistributorPuckCollection fail";
        }



    } // END TEST 2



    QThread::msleep(1000.0);

    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    QCoreApplication::quit();
    thread_logs.join();
}
