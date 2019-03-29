// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/Lidar.hpp>
#include <WestBot/RobotRock/Odometry.hpp>

#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testLidar" )
}

int main( int argc, char *argv[] )
{
    QCoreApplication app( argc, argv );

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif

    Hal::Ptr hal = std::make_shared< Hal >();

    //hal->dump();

    // set lidar pwm to 30%
    hal->_motor5Override.write(1);
    hal->_motor5Value.write(0);
    QThread::msleep( 2000 );

    Odometry odometry(hal);

    // init position
    odometry.setPosition({.x=0.0,.y=0.0,.theta=RAD(0.0)});

    LidarRPLidarA2 lidar( QString("/dev/ttyAL6"), 256000, std::make_shared< ItemRegister >( hal->_motor5Value ));

    lidar.init();
    lidar.startMotor(15.0);
    QThread::msleep( 2000 );

    lidar.startScan();

    LidarData data[LIDAR_MAX_SCAN_POINTS];
    uint32_t count;
    bool ok;

    ok = lidar.get360ScanData(data,count);

    if (!ok || count == 0)
        tFatal(LOG) << "Lidar is not getting any data";

    // impossible to get valid data with this minimum
    lidar.setMinimumQuality(64);

    ok = lidar.get360ScanData(data,count);
    if (!ok || count != 0)
        tFatal(LOG) << "Lidar setMinimumQuality is not working as expected" << ok << count;

    tInfo(LOG) << "Stopping Motor & Scan";
    lidar.stopScan();
    lidar.stopMotor();
    QThread::msleep( 6000 );


    lidar.setMinimumQuality(0);
    // we flush internal data
    ok = lidar.get360ScanData(data,count);

    ok = lidar.get360ScanData(data,count);
    if (ok || count != 0)
        tFatal(LOG) << "Lidar get360ScanData should not output data when motor is not rotating" << ok << count;

    // set back motor to 50%
    lidar.startMotor(50.0);
    QThread::msleep( 2000 );
    lidar.startScan();
    QThread::msleep( 4000 );


    // check if Odometry current position is going back through the lidar data
    RobotPos pos = {.x=10.0,.y=20.0,.theta=RAD(45.0)};

    odometry.setPosition(pos);

    // we flush internal data
    for (int i = 0;i < 10;i++)
        ok = lidar.get360ScanData(data,count);

    if (count != 0)
    {
        if (data[0].pos.x != pos.x || data[0].pos.y != pos.y  || fabs(data[0].pos.theta - pos.theta) > RAD(0.1))
            tFatal(LOG) << "Lidar robot first pos wrong data" << data[0].pos.x << data[0].pos.y << data[0].pos.theta << pos.theta;

        if (data[count-1].pos.x != pos.x || data[count-1].pos.y != pos.y  || fabs(data[count-1].pos.theta - pos.theta) > RAD(0.1))
            tFatal(LOG) << "Lidar robot last pos wrong data" << data[count-1].pos.x << data[count-1].pos.y << data[count-1].pos.theta;

    } else {
        tFatal(LOG) << "Lidar should ouput data" << ok << count;
    }

    lidar.stopScan();
    // flush
    lidar.get360ScanData(data,count);
    // check if Odometry current position is going back through the lidar data
    pos = {.x=20.0,.y=20.0,.theta=RAD(45.0)};

    odometry.setPosition(pos);

    lidar.startScan();


    // get data
    for (int i = 0;i < 1;i++)
        ok = lidar.get360ScanData(data,count);

    if (count != 0)
    {
        if (data[0].pos.x != pos.x || data[0].pos.y != pos.y  || fabs(data[0].pos.theta - pos.theta) > RAD(0.1))
            tFatal(LOG) << "Lidar single robot first pos wrong data" << data[0].pos.x << data[0].pos.y << data[0].pos.theta;

        if (data[count-1].pos.x != pos.x || data[count-1].pos.y != pos.y  || fabs(data[count-1].pos.theta - pos.theta) > RAD(0.1))
            tFatal(LOG) << "Lidar single robot last pos wrong data" << data[count-1].pos.x << data[count-1].pos.y << data[count-1].pos.theta;

    } else {
        tFatal(LOG) << "Lidar should ouput data" << ok << count;
    }



    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    //return app.exec();
}
