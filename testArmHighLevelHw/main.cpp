#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/ArmLowLevel.hpp>
#include <WestBot/RobotRock/ArmHighLevel.hpp>
#include <WestBot/RobotRock/Vl6180x.hpp>


#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testArmLowLevel" )
}



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

    Vl6180x vl6180xL("/dev/ttyAL8");
    Vl6180x vl6180xR("/dev/ttyAL13");

    hal->_motor4Override.write(1);
    hal->_motor5Override.write(1);

    hal->_motor4Value.write(0);


    RobotPos armPos;

    armPos.x = 200.0;
    armPos.y = 120.0;

    Odometry::Ptr odometryPtr = std::make_shared< Odometry >( hal );

    odometryPtr->setPosition({.x=0.0,.y=0.0,.theta=RAD(0.0)});


    ArmLowLevel::Ptr armLLPtr = std::make_shared< ArmLowLevel >();

/*
    ItemRegister::Ptr pidfirstreg = std::make_shared< ItemRegister >( hal->_pidCustom1FreqHz );
    ItemRegister::Ptr pump = std::make_shared< ItemRegister >( hal->_motor4Value );
    ItemRegister::Ptr valve = std::make_shared< ItemRegister >( hal->_output2 );

    bool isLeft = true;

    bool initOk = armLLPtr->init(hal,
               pidfirstreg,
               true,
               pump,
               valve,
               &vl6180xL,
               vl6180xL.distancePointer(0),
               SMART_SERVO_DYNAMIXEL,
               4,
               SMART_SERVO_DYNAMIXEL,
               2,
               SMART_SERVO_DYNAMIXEL,
               5,
               true,
               0.0
               );
               */


    ItemRegister::Ptr pidfirstreg = std::make_shared< ItemRegister >( hal->_pidCustom2FreqHz );
    ItemRegister::Ptr pump = std::make_shared< ItemRegister >( hal->_motor5Value );
    ItemRegister::Ptr valve = std::make_shared< ItemRegister >( hal->_output1 );

    bool isLeft = false;


    bool initOk = armLLPtr->init(hal,
               pidfirstreg,
               true,
               pump,
               valve,
               &vl6180xR,
               vl6180xR.distancePointer(0),
               SMART_SERVO_DYNAMIXEL,
               6,
               SMART_SERVO_DYNAMIXEL,
               3,
               SMART_SERVO_DYNAMIXEL,
               7,
               false,
               2.0
               );

    ///// GENERAL CONFIG /////

    //x=200.0
    //y=120.0

    //stoackage
    //x=140.0;
    //y=60.0

    unsigned int tId;
    unsigned int i;

    double inv = 1.0;

    if (isLeft == true)
        inv = -1.0;

    // TEST 1: Grab and release items
    {
        tId = 1;

        bool actionOk;

        ArmHighLevel armHL;

        armHL.init(odometryPtr,armLLPtr,isLeft);

        armHL.enable();

        //armHL.confArmPos(194.6,-118.5);
        armHL.confArmPos(197,-118.5*inv);
        //armHL.confStorage(138.0,-50.0,150.8);
        armHL.confStorage(155,-55.0*inv,150.8);

        armHL.setMode(ARM_HL_MODE_HORIZONTAL);

        armHL.moveZ(200.0);

        //armHL.setVacuum(true);

        actionOk = armHL.moveArmRel(300.0,-120.0*inv);

        if (actionOk)
        {
            actionOk = armHL.actionGroundPuckCollection(266.0,-70.0*inv);

            if (actionOk)
            {
                armHL.moveZ(220.0);

                actionOk = armHL.actionPuckStore();

                if (actionOk)
                {
                    actionOk = armHL.actionPuckUnstore();

                    if (actionOk)
                    {
                        actionOk = armHL.moveArmRel(266.0,-70.0*inv);

                        if (actionOk)
                        {
                            armHL.actionPuckRelease(266.0,-70.0*inv,50.0);
                        }
                    }
                }
            }
        }
        armHL.moveZ(220.0);

        //armHL.actionGroundPuckCollection(300.0,-160.0);

        //armHL.moveZ(220.0);

        //armHL.actionPuckStore();
/*
        armHL.actionGroundPuckCollection(266.0,-150.0*inv);

        armHL.moveZ(220.0);

        armHL.actionPuckStore();
*/
        //

        //armHL.actionPuckRelease(300.0,120.0,50.0);
        /*
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
*/
    } // END TEST 1


#if 0

    // TEST 2: Grab and release items
    {
        tId = 2;

        tInfo(LOG) << "ArmHighLevel: TEST" << tId;

        armLLPtr->resetObjects();

        ArmHighLevel armHL;

        armHL.init(odometryPtr,armLLPtr);

        armHL.enable();

        armHL.confArmPos(0.0,0.0);

        armHL.confStorage(0,25,25);

        bool ret;

        ret = armHL.actionGroundPuckCollection(50.0,50.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionGroundPuckCollection 1 failed";
        }

        ret = armHL.actionPuckStore();


        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckStore 1  failed";
        }


        ret = armHL.actionGroundPuckCollection(25.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionGroundPuckCollection 2 failed";
        }

        ret = armHL.actionPuckStore();


        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckStore 2  failed";
        }

        if (armHL.getPuckCount() != 2)
        {
            tFatal(LOG) << "Test" << tId << "getPuckCount puck count issue" << armHL.getPuckCount();
        }

        ret = armHL.actionPuckUnstore();

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckUnstore 1  failed";
        }

        ret = armHL.actionPuckRelease(50.0,50.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckRelease 1 failed";
        }



        ret = armHL.actionPuckUnstore();

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckUnstore 2  failed";
        }

        ret = armHL.actionPuckRelease(25.0,25.0,25.0);

        if (!ret)
        {
            tFatal(LOG) << "Test" << tId << "actionPuckRelease 2 failed";
        }

        if (armHL.getPuckCount() != 0)
        {
            tFatal(LOG) << "Test" << tId << "getPuckCount released puck count issue" << armHL.getPuckCount();
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

#endif

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
