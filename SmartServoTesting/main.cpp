// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/SmartServo.hpp>


using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SmartServoTesting" )
}

//#define DEBUG
#define SIMU


#define TEST_COUNT 20000
#define GET_POS_DELAY 20

Hal::Ptr hal = std::make_shared< Hal >();

void test_servo(const QString& name, uint8_t protocol, uint8_t budId){
    tInfo( LOG ) << "begin" << __FUNCTION__ << name;

    SmartServo s(name);
    s.attach(hal,protocol,budId);

    if (protocol == SMART_SERVO_DYNAMIXEL)
        s.setRawWrite8(DYNAMIXEL_REGS_P,64);

    s.enable();

    int i;
    for (i=0;i<TEST_COUNT;i++)
    {
        s.setPositionAndSpeed(false,false,i*10%1000,0);

        while (s.moving()){
            QThread::msleep( GET_POS_DELAY );
        }
    }

    tInfo( LOG ) << "end " << __FUNCTION__ << name;
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

int main(int argc, char *argv[])
{
    std::thread thread_logs(log_management,argc,argv);

    // Only to disable the Debug Logs
    SmartServo sDebug("");
    sDebug.LOG().enableDebug(false);

    /*
    SmartServo s("s feetech");
    s.attach(hal,SMART_SERVO_FEETECH,1);
    s.changeId(2);
    s.setPosition(false,false,500);
    tInfo( LOG ) << "DO WHAT YOU WANT...";
    */
/*
    SmartServo s3("Test");
    s3.attach(hal,SMART_SERVO_FEETECH,2);
    s3.enable();
    tInfo( LOG ) << "1";

    s3.setPositionAndSpeed(false,false,0,150);
    tInfo( LOG ) << "2";
    while (s3.moving());

    s3.setPositionAndSpeed(false,false,500,150);
    tInfo( LOG ) << "3";
    while (s3.moving());

    s3.setPositionAndSpeed(false,false,800,150);
    tInfo( LOG ) << "test";
    while (s3.moving());

    thread_logs.join();
    while(1);
*/
/*
    SmartServo s2("Test");
    s2.attach(hal,SMART_SERVO_FEETECH,2);
    s2.enable();
    tInfo( LOG ) << "1";

    s2.setPositionAndSpeed(false,false,500,150);
    tInfo( LOG ) << "2";
    while (s2.moving());


    SmartServo s3("Test");
    s3.attach(hal,SMART_SERVO_FEETECH,1);
    s3.enable();
    tInfo( LOG ) << "1";

    s3.setPositionAndSpeed(false,false,500,150);
    tInfo( LOG ) << "2";
    while (s3.moving());
*/
    /*
    SmartServo s1("Test");
    s1.attach(hal,SMART_SERVO_DYNAMIXEL,1);
    tInfo( LOG ) << s1.getRawRead8(DYNAMIXEL_REGS_HW_ERROR_STATUS);
*/
    SmartServo s4("Test");
    s4.attach(hal,SMART_SERVO_FEETECH,2);
    //s4.attach(hal,SMART_SERVO_DYNAMIXEL,1);
    //s4.setRawWrite8(DYNAMIXEL_REGS_P,255);

    s4.enable();
    tInfo( LOG ) << "1";

    bool pos = false;
    while (1)
    {
        if (pos == false)
            s4.setPositionAndSpeed(false,false,0,0);
        else
            s4.setPositionAndSpeed(false,false,1023,0);

        while (s4.moving());
        pos = !pos;
    }

    /*

    s3.setPositionAndSpeed(false,false,500,150);
    tInfo( LOG ) << "3";
    while (s3.moving());

    s3.setPositionAndSpeed(false,false,800,150);
    tInfo( LOG ) << "test";
    while (s3.moving());
*/
    thread_logs.join();
    while(1);


    /*
    std::thread thread_test_feetech1(test_servo,"feetech 1",SMART_SERVO_FEETECH,1);
    std::thread thread_test_feetech2(test_servo,"feetech 2",SMART_SERVO_FEETECH,2);

    std::thread thread_test_dyna1(test_servo,"dyna 1",SMART_SERVO_DYNAMIXEL,1);
    std::thread thread_test_dyna2(test_servo,"dyna 2",SMART_SERVO_DYNAMIXEL,2);

    thread_test_feetech1.join();
    thread_test_feetech2.join();
    thread_test_dyna1.join();
    thread_test_dyna2.join();
*/


    tInfo( LOG ) << "SUCCESS ";

}
