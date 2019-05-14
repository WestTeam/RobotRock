// Copyright (c) 2019 All Rights Reserved WestBot

#include <QCoreApplication>
#include <QHostAddress>

#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include "WestBot/RobotRock/SimTcpServer.hpp"

#include "WestBot/RobotRock/TrajectoryManagerSimu.hpp"
#include "WestBot/RobotRock/OdometrySimu.hpp"


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

    SimData _data;

    OdometrySimu::Ptr _odometry = std::make_shared< OdometrySimu >();

    _odometry->setPosition({10,20,RAD(15.0)});

    TrajectoryManagerSimu _traj(_odometry);

    _traj.init();

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

    float x = 20.0;

    do {
        QThread::msleep(100);

        x+=0.01;
        _traj.moveForwardToXYAbs(0.0,x,0.0,true);

        RobotPos pos = _odometry->getPosition();

        _data.objectId = 0;
        _data.objectPos.x = pos.x;
        _data.objectPos.y = pos.y;
        _data.objectPos.theta = pos.theta;
        _data.objectType = 0;
        _data.objectColor = 0;
        _data.objectSize = 100.0;
        _data.objectMode = 0;

        _simServer.updateClients(_data);

        app.processEvents();
    } while (true);

    return app.exec();
}
