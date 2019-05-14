#include <QCoreApplication>
#include <QThread>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/OdometryHw.hpp>

#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testOdometry" )
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

    OdometryHw odometry(hal);

    RobotPos pos;

    for (int i = 0; i  < 1000; i++)
    {
        pos.x = (double)(rand()%3200);
        if (rand()%2 == 1)
            pos.x *= -1;
        pos.y = (double)(rand()%1500);
        if (rand()%2 == 1)
            pos.y *= -1;
        pos.theta = (double)(rand()%313)/100.0;
        if (rand()%2 == 1)
            pos.theta *= -1;

        tDebug(LOG) << "New Position Check: " << pos.x << pos.y << pos.theta;

        odometry.setPosition(pos);
        RobotPos curPos = odometry.getPosition();

        if (!(curPos == pos))
        {
            tDebug(LOG) << "ERROR setPosition/getPosition" << curPos.x << pos.x << curPos.y << pos.y << curPos.theta << pos.theta;
        }
    }

    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    /*
    exit(0);
    double t = round(0.499);
    tDebug(LOG) << (int16_t)t;

    pos.x = 19.0;
    pos.y = 200;
    pos.theta = -1.9;

    odometry.setPosition(pos);
    QThread::msleep( 100 );
    RobotPos curPos = odometry.getPosition();

    tDebug(LOG) << (int16_t)(DEG((double)curPos.theta)*10.0) << (int16_t)(DEG((double)pos.theta)*10.0);
    tDebug(LOG) << "ERROR setPosition/getPosition" << curPos.theta << pos.theta;

    if (!(curPos == pos))
    {
        tDebug(LOG) << (int16_t)(DEG((double)curPos.theta)*10.0) << (int16_t)(DEG((double)pos.theta)*10.0);
        tFatal(LOG) << "ERROR setPosition/getPosition" << curPos.theta << pos.theta;
    }*/

    //return app.exec();
}
