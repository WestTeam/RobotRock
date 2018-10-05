// Copyright (c) 2018 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "main" )
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

    tInfo( LOG ) << "Info: Hello WestBot !";

    Hal hal;
    SystemManager system( hal );

    if( ! system.init() )
    {
        tWarning( LOG ) << "Failed to init system manager";
        return EXIT_FAILURE;
    }

    return app.exec();
}
