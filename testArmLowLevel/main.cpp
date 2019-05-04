#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/SmartServo.hpp>
#include <WestBot/RobotRock/ArmLowLevel.hpp>


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


    ArmLowLevel armLL;
    uint32_t dist = 0;

    hal->_outputOverride.write( 0x01010101 );

    hal->_motor4Override.write(1);
    hal->_motor4Value.write(0);


    hal->_output1.write(0);

    QThread::msleep( 2000000 );
    while(1);

    bool initOk = armLL.init(hal,
               std::make_shared< ItemRegister >( hal->_motor4Value ),
               std::make_shared< ItemRegister >( hal->_output1 ),
               &dist,
               SMART_SERVO_DYNAMIXEL,
               6,
               SMART_SERVO_DYNAMIXEL,
               3,
               SMART_SERVO_DYNAMIXEL,
               2
               );


    if (!initOk)
    {
        tFatal(LOG) << "testArmLowLevel: Init KO";
    }


    ///// GENERAL CONFIG /////


    unsigned int tId;
    unsigned int i;

    // TEST 1: Test Z only
    {
        tId = 1;

        armLL.disable();

        armLL.setZSpeed(10.0);
        armLL.setZAcc(1.0);

        armLL.enableZ(true);

        i = 0;

        double zTarget = 300.0;

        // check first if init ok & target speed ok
        do
        {
            armLL.setZ(zTarget);

            zTarget -= 10.0;

            bool waitOk = armLL.waitZTargetOk(1000);

            if (waitOk == false)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": waitZTargetOk timeout, (target/pos)" << zTarget << armLL.getZ();
            }

        } while (zTarget > 100.0);


        // check first if init ok & target speed ok
        do
        {
            armLL.setZ(zTarget);

            zTarget -= 10.0;

            bool waitOk = armLL.waitZTargetOk(1000);

            if (waitOk == false)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": waitZTargetOk timeout, (target/pos)" << zTarget << armLL.getZ();
            }

        } while (zTarget > 100.0);

    } // END TEST 1


/*
    // TEST 2: Test Angles Only
    {
        tId = 2;

        armLL.disable();

        armLL.enableServo(ARM_LL_SERVO_UPPER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_LOWER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_WRIST,true);

        armLL.setServoPos(ARM_LL_SERVO_UPPER_ARM,20.0);
        armLL.setServoPos(ARM_LL_SERVO_LOWER_ARM,-20.0);
        armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);

        bool waitOk = armLL.waitServosTargetOk(1000);
        QThread::msleep( 2000 );


        if (waitOk == false)
        {
            tWarning(LOG) << "testRecalage: Test " << tId << ": waitServosTargetOk timeout, (target/pos)"
                        << armLL.getServoPos(ARM_LL_SERVO_UPPER_ARM) << armLL.getServoPos(ARM_LL_SERVO_LOWER_ARM)
                        << armLL.getServoPos(ARM_LL_SERVO_WRIST);
        }

        i = 0;

        // check first if init ok & target speed ok
        do
        {
            if (i%2==0)
            {
                armLL.setServosPos(-20.0,20.0,0.0);
            } else {
                armLL.setServosPos(20.0,-20.0,-90.0);
            }
            //QThread::msleep( 2000 );


            waitOk = armLL.waitServosTargetOk(1000);

            if (waitOk == false)
            {
                tWarning(LOG) << "testRecalage: Test " << tId << ": waitServosTargetOk timeout, pos: "
                            << armLL.getServoPos(ARM_LL_SERVO_UPPER_ARM) << armLL.getServoPos(ARM_LL_SERVO_LOWER_ARM)
                            << armLL.getServoPos(ARM_LL_SERVO_WRIST);
            }

            i++;

        } while (i < 4);


    } // END TEST 2
    */


    QThread::msleep( 1000 );

    // TEST 3: Test Aspiration
    {
        tId = 3;

        armLL.disable();

        armLL.setZSpeed(10.0);
        armLL.setZAcc(1.0);

        armLL.enableZ(true);

        armLL.setZ(75.0);

        armLL.enableServo(ARM_LL_SERVO_UPPER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_LOWER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_WRIST,true);

        armLL.setServoPos(ARM_LL_SERVO_UPPER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_LOWER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);



        bool waitOk = armLL.waitServosTargetOk(1000) && armLL.waitZTargetOk(1000);

        if (waitOk == false)
        {
            tFatal(LOG) << "testRecalage: Test " << tId << ": Z or Servos timeout, (target/pos)"
                        << armLL.getZ() << armLL.getServoPos(ARM_LL_SERVO_UPPER_ARM) << armLL.getServoPos(ARM_LL_SERVO_LOWER_ARM)
                        << armLL.getServoPos(ARM_LL_SERVO_WRIST);
        }




        i = 0;

        // check first if init ok & target speed ok
        do
        {
            waitOk = true;

            armLL.setZ(75.0);
            waitOk &= armLL.waitZTargetOk(1000);

            armLL.setZ(50.0);
            waitOk &= armLL.waitZTargetOk(0);
            //QThread::msleep( 2000 );
            armLL.setZ(75.0);
            armLL.setServoPos(ARM_LL_SERVO_WRIST,0.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            QThread::msleep( 1000 );

            armLL.setVacuumPower(100.0);
            armLL.setVacuumValve(false);

            armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            QThread::msleep( 3000 );

            armLL.setServoPos(ARM_LL_SERVO_WRIST,0.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            QThread::msleep( 100 );

            armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            QThread::msleep( 500 );

            armLL.setZ(63.0);
            waitOk &= armLL.waitZTargetOk(1000);
            //armLL.setVacuumPower(0.0);
            armLL.setVacuumValve(true);
            QThread::msleep( 1000 );


            if (waitOk == false)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": waitOk failure!";
            }

            i++;

        } while (i < 10);


    } // END TEST 3

#ifdef hh

    // TEST 4: Test Aspiration
    {
        tId = 4;

        armLL.disable();

        armLL.setZSpeed(10.0);
        armLL.setZAcc(1.0);

        armLL.enableZ(true);

        armLL.setZ(75.0);

        armLL.enableServo(ARM_LL_SERVO_UPPER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_LOWER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_WRIST,true);

        armLL.setServoPos(ARM_LL_SERVO_UPPER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_LOWER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);



        bool waitOk = armLL.waitServosTargetOk(1000) && armLL.waitZTargetOk(1000);

        if (waitOk == false)
        {
            tFatal(LOG) << "testRecalage: Test " << tId << ": Z or Servos timeout, (target/pos)"
                        << armLL.getZ() << armLL.getServoPos(ARM_LL_SERVO_UPPER_ARM) << armLL.getServoPos(ARM_LL_SERVO_LOWER_ARM)
                        << armLL.getServoPos(ARM_LL_SERVO_WRIST);
        }




        i = 0;

        // check first if init ok & target speed ok
        do
        {
            waitOk = true;

            armLL.setZ(75.0);
            waitOk &= armLL.waitZTargetOk(1000);

            double dist = armLL.getDistance();
            if (dist > 50)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": no Object found failure! dist:" << dist;
            }

            armLL.setVacuumPower(100.0);
            armLL.setVacuumValve(false);
            armLL.setZ(50.0);
            waitOk &= armLL.waitZTargetOk(0);
            QThread::msleep( 500 );

            dist = armLL.getDistance();

            if (dist > 25)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": catching failure! dist:" << dist;
            }

            armLL.setZ(75.0);
            armLL.setServoPos(ARM_LL_SERVO_WRIST,0.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            armLL.setServoPos(ARM_LL_SERVO_WRIST,0.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            armLL.setServoPos(ARM_LL_SERVO_WRIST,-90.0);
            waitOk &= armLL.waitServoTargetOk(ARM_LL_SERVO_WRIST,0);
            armLL.setZ(63.0);
            waitOk &= armLL.waitZTargetOk(1000);
            armLL.setVacuumPower(0.0);
            armLL.setVacuumValve(true);
            QThread::msleep( 200 );

            dist = armLL.getDistance();
            if (dist <= 25)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": release failure! dist:" << dist;
            }

            if (waitOk == false)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": waitOk failure!";
            }

            i++;

        } while (i < 10);


    } // END TEST 4
#endif



    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    QCoreApplication::quit();
    thread_logs.join();
}
