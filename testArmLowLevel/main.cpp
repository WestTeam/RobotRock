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
/*
    hal->_resetAll.write(1);
    hal->_resetAll.write(0);
*/

    Vl6180x vl6180xL("/dev/ttyAL8");
    Vl6180x vl6180xR("/dev/ttyAL13");


    ArmLowLevel armLL;
    ArmLowLevel armLLR;

    uint32_t dist = 0;
    hal->_outputOverride.write( 0x01010101 );

    hal->_motor4Override.write(1);
    hal->_motor4Value.write(0);

    hal->_motor5Override.write(1);
    hal->_motor5Value.write(0);

    hal->_motor3Override.write(0);
    hal->_motor3Value.write(7000);

    hal->_motor2Override.write(0);

    hal->_pwmCustom0Value.write(1000);
    hal->_pwmCustom1Value.write(1000);

    hal->_pwmCustom2Value.write(1000);

    //while(1);
/*
    hal->_motor2Override.write(1);
    hal->_motor2Value.write(10000);


    QThread::msleep(1000.0);
    hal->_motor2Override.write(1);
    hal->_motor2Value.write(0);
*/
    //while(1);



    //float max = 3.7/12.0*32768.0;
    //float value = percentage/100.0*max;
/*
    while(1){
        QThread::msleep(50.0);
        tInfo(LOG) << hal->_qei0CntValue.read<uint16_t>() << hal->_qei1CntValue.read<uint16_t>();
        tInfo(LOG) << "left" << vl6180xL.distance(0) << vl6180xL.distance(1);
        tInfo(LOG) << "right" << vl6180xR.distance(0) << vl6180xR.distance(1);


       // hal->_output1.write(1);
        QThread::msleep(50.0);
       // hal->_output1.write(0);
    }
    */

    hal->_motor4Value.write(0);

    ItemRegister::Ptr pidfirstregLeft = std::make_shared< ItemRegister >( hal->_pidCustom1FreqHz );
    ItemRegister::Ptr pumpLeft = std::make_shared< ItemRegister >( hal->_motor4Value );
    ItemRegister::Ptr valveLeft = std::make_shared< ItemRegister >( hal->_output2 );


    bool initOk;

    /*
    initOk = armLL.init(hal,
               pidfirstregLeft,
               true,
               pumpLeft,
               valveLeft,
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

    ItemRegister::Ptr pidfirstregRight = std::make_shared< ItemRegister >( hal->_pidCustom2FreqHz );
    ItemRegister::Ptr pumpRight = std::make_shared< ItemRegister >( hal->_motor5Value );
    ItemRegister::Ptr valveRight = std::make_shared< ItemRegister >( hal->_output1 );


    initOk = armLLR.init(hal,
               pidfirstregRight,
               true,
               pumpRight,
               valveRight,
               &vl6180xR,
               vl6180xR.distancePointer(0),
               SMART_SERVO_DYNAMIXEL,
               6,
               SMART_SERVO_DYNAMIXEL,
               3,
               SMART_SERVO_DYNAMIXEL,
               7,
               false,
               -0.2
            );


    //hal->_motor4Value.write(0);
    while(1);

    if (!initOk)
    {
        tWarning(LOG) << "testArmLowLevel: Init KO";
    }

    //while(1);
    /*tDebug(LOG) << armLL.getDistance() << *vl6180xL.distancePointer(0) << vl6180xL.status(0);
    tDebug(LOG) << armLL.getDistance() << *vl6180xL.distancePointer(0) << vl6180xL.status(0);

    tDebug(LOG) << armLL.getDistance() << *vl6180xL.distancePointer(1) << vl6180xL.status(1);
    tDebug(LOG) << armLL.getDistance() << *vl6180xL.distancePointer(1) << vl6180xL.status(1);
*/
    //while (1);

    /*
    armLL.setZ(200.0);
    armLL.waitZTargetOk(0);
  */
  //armLL.setZ(150.0);
  //  armLL.waitZTargetOk(0);
    //armLL.setZ(300.0);
    //armLL.waitZTargetOk(0);



    //while(1);

    ///// GENERAL CONFIG /////

#define OFFSET_DEBUG 18.5

    unsigned int tId;
    unsigned int i;

    // TEST 1: Test Z only
    {
        tId = 1;

        armLL.disable();

        //armLL.setZSpeed(10.0);
        //armLL.setZAcc(1.0);

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
                tWarning(LOG) << "testRecalage: Test " << tId << ": waitZTargetOk timeout, (target/pos)" << zTarget << armLL.getZ();
            }

        } while (zTarget > 100.0+OFFSET_DEBUG);


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
#if 0
    // TEST 3: Test Aspiration
    {
        tId = 3;

        armLL.disable();

        //armLL.setZSpeed(10.0);
        //armLL.setZAcc(1.0);

        armLL.enableZ(true);

        armLL.setZ(75.0+OFFSET_DEBUG);

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

            armLL.setZ(75.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(1000);

            armLL.setZ(50.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(0);
            //QThread::msleep( 2000 );
            armLL.setZ(75.0+OFFSET_DEBUG);
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

            armLL.setZ(63.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(1000);
            armLL.setVacuumPower(0.0);
            armLL.setVacuumValve(true);
            QThread::msleep( 1000 );


            if (waitOk == false)
            {
                tFatal(LOG) << "testRecalage: Test " << tId << ": waitOk failure!";
            }

            i++;

        } while (i < 3);


    } // END TEST 3

#endif

    // TEST 4: Test Aspiration
    {
        tId = 4;

        armLL.disable();

        //armLL.setZSpeed(200.0);
        //armLL.setZAcc(1.0);

        armLL.enableZ(true);

        armLL.setZ(75.0+OFFSET_DEBUG);

        armLL.enableServo(ARM_LL_SERVO_UPPER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_LOWER_ARM,true);
        armLL.enableServo(ARM_LL_SERVO_WRIST,true);

        armLL.setServoPos(ARM_LL_SERVO_UPPER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_LOWER_ARM,0.0);
        armLL.setServoPos(ARM_LL_SERVO_WRIST,90.0);



        bool waitOk = armLL.waitServosTargetOk(1000) && armLL.waitZTargetOk(1000);

        if (waitOk == false)
        {
            tWarning(LOG) << "testRecalage: Test " << tId << ": Z or Servos timeout, (target/pos)"
                        << armLL.getZ() << armLL.getServoPos(ARM_LL_SERVO_UPPER_ARM) << armLL.getServoPos(ARM_LL_SERVO_LOWER_ARM)
                        << armLL.getServoPos(ARM_LL_SERVO_WRIST);
        }




        i = 0;

        // check first if init ok & target speed ok
        do
        {
            waitOk = true;

            armLL.setZ(75.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(1000);

            double dist = armLL.getDistance();
            if (dist > 50)
            {
                tWarning(LOG) << "testRecalage: Test " << tId << ": no Object found failure! dist:" << dist;
            }

            armLL.setVacuumPower(100.0);
            armLL.setVacuumValve(false);
            armLL.setZ(50.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(0);
            QThread::msleep( 1000 );

            dist = armLL.getDistance();

            if (dist > 25)
            {
                tWarning(LOG) << "testRecalage: Test " << tId << ": catching failure! dist:" << dist;
            }

            armLL.setZ(120.0+OFFSET_DEBUG);

            waitOk &= armLL.waitZTargetOk(1000);

            QThread::msleep( 2000 );

            armLL.setZ(80.0+OFFSET_DEBUG);
            waitOk &= armLL.waitZTargetOk(1000);
            armLL.setVacuumPower(0.0);
            armLL.setVacuumValve(true);
            QThread::msleep( 800 );

            dist = armLL.getDistance();
            if (dist <= 25)
            {
                tWarning(LOG) << "testRecalage: Test " << tId << ": release failure! dist:" << dist;
            }

            if (waitOk == false)
            {
                tWarning(LOG) << "testRecalage: Test " << tId << ": waitOk failure!";
            }

            i++;

        } while (i < 3);

        armLL.setZ(300.0);
        armLL.waitZTargetOk(0);
    } // END TEST 4



    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    QCoreApplication::quit();
    thread_logs.join();
}
