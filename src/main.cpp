// Copyright (c) 2018 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

#define DEBUG
#define SIMU

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

    Hal hal;
    SystemManager system( hal );

    if( ! system.init() )
    {
        tWarning( LOG ) << "Failed to init system manager";
        return -1;
    }

#ifdef DEBUG
    hal.dump();
#endif

#ifdef SIMU
    hal._modeSimu.write( 1 );
#endif

    tInfo( LOG ) << "==== System ready ! ==== ";

    return app.exec();
}
