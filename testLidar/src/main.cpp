// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/Lidar.hpp>

#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.TestLidar" )
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

    hal->dump();

    // set lidar pwm to 30%
    hal->_motor5Override.write(1);
    hal->_motor5Value.write(10000);
    QThread::msleep( 2000 );


    Lidar lidar( "/dev/ttyAL6" );

    if( ! lidar.init() )
    {
        tWarning( LOG ) << "Failed to init lidar module";
        return false;
    }

    lidar.startScan();
    lidar.ascendScanData();
    lidar.stopScan();

    return app.exec();
}
