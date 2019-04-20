// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>
#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Vl6180x.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testVl6180x" )
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Handler handler( a.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif

    tInfo( LOG ) << "==== Test ready ! ==== ";

    Vl6180x vl6180x;

    while( 1 )
    {
        tDebug( LOG ) << "Read distance:" << vl6180x.distance();
        a.processEvents();
    }

    return a.exec();
}
