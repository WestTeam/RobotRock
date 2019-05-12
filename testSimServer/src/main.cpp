// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>
#include <QHostAddress>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include "WestBot/RobotRock/SimTcpServer.hpp"

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.TestSimServer" )

    const int DEFAULT_SIM_PORT = 4242;
}

int main( int argc, char *argv[] )
{
    QCoreApplication app( argc, argv );

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

    handler.setEnableDebugLevel( true );

    tInfo( LOG ) << "DO WHAT YOU WANT...";

    SimTcpServer _simServer;

    if( ! _simServer.listen( QHostAddress::Any, DEFAULT_SIM_PORT ) )
    {
       tWarning( LOG )
           << "Unable to start the server:"
           << _simServer.errorString();
    }
    else
    {
        _simServer.showConnectionInformation();
    }

    return app.exec();
}
