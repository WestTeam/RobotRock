// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/SystemManagerSimu.hpp>

#include "StrategyManagerHomologation.hpp"

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Main" )
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

    StrategyManagerHomologation::Ptr strategy = std::make_shared< StrategyManagerHomologation >();
    SystemManagerSimu system( strategy );

    tInfo( LOG ) << "==== System started ! ==== ";

    if( ! system.init() )
    {
        tFatal( LOG ) << "Failed to init system manager";
    }

    tInfo( LOG ) << "==== System ready ! ==== ";

    return app.exec();
}
