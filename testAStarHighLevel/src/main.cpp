// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QCoreApplication>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/AStarHighLevel.hpp>

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

    tInfo( LOG ) << "==== Test ready ! ==== ";

    AStarHighLevel astar( 60, 40 );

    astar.setCurrentPos( 0, 0 );
    astar.setTarget( 59, 39 );
    astar.setObstacle( 5, 5, 20, 20 );

    astar.dumpMap();

    astar.processCurrentRoute(false);

    astar.dumpMap();

    return app.exec();
}
