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


#define TEST_COUNT 20
#define GET_POS_DELAY 2
#define DEV_ID_MAX 16

Hal::Ptr hal = std::make_shared< Hal >();



void scan_bus(uint8_t protocol, std::list<int> & devList)
{
    tInfo( LOG ) << "begin" << __FUNCTION__ << protocol;

    for (int i=1;i<=DEV_ID_MAX;i++)
    {
        SmartServo s("protocol:"+ QString::number(protocol) + " bus:" + QString::number(i));

        try {
            s.attach(hal,protocol,i);
            s.getPosition(true);
            devList.push_back(i);
        } catch (...) {
        }
    }

    tInfo( LOG ) << "end " << __FUNCTION__ << protocol;
}



void check_config(uint8_t protocol, std::list<int> & devList)
{
    tInfo( LOG ) << "begin" << __FUNCTION__ << protocol;

    bool id1Changed = false;

    for (int d : devList)
    {
        SmartServo s("protocol:"+ QString::number(protocol) + " bus:" + QString::number(d));

        try {
            s.attach(hal,protocol,d);

            if (protocol == SMART_SERVO_DYNAMIXEL)
            {
                s.setRawWrite8(DYNAMIXEL_REGS_TORQUE_ENABLE,0);

                if (s.getRawRead8(DYNAMIXEL_REGS_RETURN_DELAY_TIME) != 0)
                {
                    tInfo( LOG ) << "[DYNAMIXEL] Changing Delay Time from " << s.getRawRead8(DYNAMIXEL_REGS_RETURN_DELAY_TIME) << "to" << 0;
                    s.setRawWrite8(DYNAMIXEL_REGS_RETURN_DELAY_TIME,0);

                }
                if (s.getRawRead8(DYNAMIXEL_REGS_ALARM_SHUTDOWN) != 2)
                {
                    tInfo( LOG ) << "[DYNAMIXEL] Alarm Shutdown from " << s.getRawRead8(DYNAMIXEL_REGS_RETURN_DELAY_TIME) << "to" << 2;
                    s.setRawWrite8(DYNAMIXEL_REGS_ALARM_SHUTDOWN,2);

                }

                s.setRawWrite8(DYNAMIXEL_REGS_TORQUE_ENABLE,1);
            }



            if  (d == 1)
            {
                tInfo( LOG ) << "Changing bus Id " << d << "to" << devList.back()+1;

                s.changeId(devList.back()+1);
                id1Changed = true;
            }


        } catch (...) {}
    }

    if (id1Changed)
    {
        devList.push_back(devList.back()+1);
        devList.pop_front();
    }
    tInfo( LOG ) << "end " << __FUNCTION__ << protocol;
}


void test_servo_increment(const QString& name, uint8_t protocol, uint8_t busId){
    tInfo( LOG ) << "begin" << __FUNCTION__ << name;

    SmartServo s(name);
    s.attach(hal,protocol,busId);

//    if (protocol == SMART_SERVO_DYNAMIXEL)
//        s.setRawWrite8(DYNAMIXEL_REGS_P,64);

    s.enable();

    #define INCREMENT 25

    for (int i=0;i<TEST_COUNT*1000/INCREMENT;i++)
    {
        s.setPositionAndSpeed(false,false,i*INCREMENT%1000,0);

        while (s.moving()){
            QThread::msleep( GET_POS_DELAY );
            //tDebug( LOG ) << "pos " << s.getPosition(true);
        }
    }

    tInfo( LOG ) << "end " << __FUNCTION__ << name;
}


void test_servo_full_range(const QString& name, uint8_t protocol, uint8_t busId){
    tInfo( LOG ) << "begin" << __FUNCTION__ << name;

    SmartServo s(name);
    s.attach(hal,protocol,busId);
/*
    tInfo( LOG ) << name << "delay " << s.getRawRead8(DYNAMIXEL_REGS_RETURN_DELAY_TIME);
    tInfo( LOG ) << name << "firmware " << s.getRawRead16(0);
    tInfo( LOG ) << name << "model " << s.getRawRead8(2);
    tInfo( LOG ) << name << "alarm shutdown " << s.getRawRead8(DYNAMIXEL_REGS_ALARM_SHUTDOWN);
    tInfo( LOG ) << name << "hw error" << s.getRawRead8(DYNAMIXEL_REGS_HW_ERROR_STATUS);
    tInfo( LOG ) << name << "torque limit" << s.getRawRead16(DYNAMIXEL_REGS_GOAL_TORQUE_L);
    tInfo( LOG ) << name << "temp" << s.getRawRead8(DYNAMIXEL_REGS_PRESENT_TEMP);
    tInfo( LOG ) << name << "voltage" << s.getRawRead8(DYNAMIXEL_REGS_PRESENT_VOLTAGE);
    tInfo( LOG ) << name << "max temp" << s.getRawRead8(DYNAMIXEL_REGS_LIMIT_TEMP);
*/

    s.enable();

    bool dir = false;
    for (int i=0;i<TEST_COUNT;i++)
    {
        if (dir == false)
            s.setPositionAndSpeed(false,false,0,2047);
        else
            s.setPositionAndSpeed(false,false,1023,2047);

        while (s.moving());
        dir = !dir;
    }

    tInfo( LOG ) << "end " << __FUNCTION__ << name;
}


void test_servo_protocol(const QString& name, uint8_t protocol, uint8_t busId){
    tInfo( LOG ) << "begin" << __FUNCTION__ << name;

    SmartServo s(name);
    s.attach(hal,protocol,busId);

    s.enable();

    bool dir = false;
    for (int i=0;i<TEST_COUNT;i++)
    {
        uint16_t pos = s.getPosition(true);
        s.checkStatus();
        s.setPosition(true,false,100);
        s.setAction(false);
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

    std::list<int> devList[2];

    scan_bus(SMART_SERVO_DYNAMIXEL, devList[SMART_SERVO_DYNAMIXEL]);
    for (int n : devList[SMART_SERVO_DYNAMIXEL]) {
        tInfo( LOG ) << "[DYNAMIXEL] Device found with id " << n;
    }
    check_config(SMART_SERVO_DYNAMIXEL,devList[SMART_SERVO_DYNAMIXEL]);

    scan_bus(SMART_SERVO_FEETECH, devList[SMART_SERVO_FEETECH]);
    for (int n : devList[SMART_SERVO_FEETECH]) {
        tInfo( LOG ) << "[FEETECH] Device found with id " << n;
    }
    check_config(SMART_SERVO_FEETECH,devList[SMART_SERVO_FEETECH]);

    typedef void (*TestFunction) (const QString& name, uint8_t protocol, uint8_t busId);
    TestFunction functions[] =
    {
        test_servo_protocol,
        test_servo_full_range,
        test_servo_increment

    };

    for (int f=0;f<sizeof(functions)/sizeof(TestFunction);f++)
    {
        std::list<std::thread*> thread_list;

        tInfo( LOG ) << "Launch Test " <<  f;
        for (int p=0;p<2;p++)
        {

            for (int d : devList[p]) {
                std::thread* test_thread= new std::thread(functions[f],QString::number(p)+":"+QString::number(d),p,d);
                thread_list.push_back(test_thread);
            }
        }
        for (std::thread *t : thread_list) {
            t->join();
            delete(t);
        }

    }

    tInfo( LOG ) << "SUCCESS ";
    QThread::msleep( 100 );

    QCoreApplication::quit();

    thread_logs.join();
}
