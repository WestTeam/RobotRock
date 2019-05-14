#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>


#include <WestBot/HumanAfterAll/Category.hpp>
#include <WestBot/HumanAfterAll/ConsoleAppender.hpp>
#include <WestBot/HumanAfterAll/Handler.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/OdometryHw.hpp>
#include <WestBot/RobotRock/Lidar.hpp>
#include <WestBot/RobotRock/Recalage.hpp>
#include <WestBot/RobotRock/TrajectoryManagerHw.hpp>



#define DEBUG
#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

using namespace WestBot::HumanAfterAll::Logging;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testRecalageLive" )
}

void log_management(int argc, char *argv[]){
    QCoreApplication app(argc,argv);

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif

    app.exec();
}

int main( int argc, char *argv[] )
{
    std::thread thread_logs(log_management,argc,argv);
/*
    QCoreApplication app( argc, argv );

    Handler handler( app.instance() );
    ConsoleAppender consoleAppender;
    handler.addAppender( & consoleAppender );

#ifdef DEBUG
    handler.setEnableDebugLevel( true );
#endif
*/


    Hal::Ptr hal = std::make_shared< Hal >();

    hal->_motor5Override.write(1);
    hal->_motor5Value.write(0);

    ///// GENERAL CONFIG /////

    RobotPos rpos; // real position
    RobotPos rpos_with_error; // position currently in odo (with error)
    RobotPos lidarPos = {.x = 206.08, .y = 101.1, .theta = -0.118+RAD(7.5)};// lidar position from robot center

    Odometry::Ptr odometryPtr = std::make_shared< OdometryHw >( hal );
    //LidarTest::Ptr lidarTestPtr;// = std::make_shared< LidarTest >(odometryPtr);


    // Config PID Distance
    hal->_pidDistanceEnable.write( 0 );
    hal->_pidDistanceOverride.write( 0 );
    hal->_pidDistanceInverted.write( 0 );
    hal->_pidDistanceKp.write( ( float ) 2000.0 );
    hal->_pidDistanceKi.write( ( float ) 0.0 );
    hal->_pidDistanceKd.write( ( float ) 0.0 );

    // We set it but this is override by the TrajectoryManager
    hal->_pidDistanceSpeed.write( ( float ) 0.01 );
    hal->_pidDistanceAcceleration.write( ( float ) 0.0001 );
    hal->_pidDistanceSaturation.write( 25000 );

    hal->_pidDistanceTarget.write( hal->_pidDistancePosition.read< float >() );
    hal->_pidDistanceEnable.write( 1 );

    // Config PID Angle
    hal->_pidAngleEnable.write( 0 );
    hal->_pidAngleOverride.write( 0 );
    hal->_pidAngleInverted.write( 1 );
    hal->_pidAngleKp.write( ( float ) 500000.0 );
    hal->_pidAngleKi.write( ( float ) 0.0 );
    hal->_pidAngleKd.write( ( float ) 0.0 );

    hal->_pidAngleSpeed.write( ( float ) 0.0001 );
    hal->_pidAngleAcceleration.write( ( float ) 0.00000002 );
    hal->_pidAngleSaturation.write( 25000 );

    hal->_pidAngleTarget.write( hal->_pidAnglePosition.read< float >() );
    hal->_pidAngleEnable.write( 1 );





/*
    lidarTestPtr->borderListAdd(0,0   ,-1500,0   ,1500);
    lidarTestPtr->borderListAdd(1,0   ,1500 ,2000,1500);
    lidarTestPtr->borderListAdd(0,2000,1500 ,2000,-1500);
    lidarTestPtr->borderListAdd(1,2000,-1500,0   ,-1500);
*/
    unsigned int tId;
    unsigned int i;
    double targetHz;

    // TEST 5: Test InitDone with RPLIDAR
    {
        tId = 5;

        rpos = {.x = 45.0, .y = -350, .theta = RAD(0)};
        // 22.5 / 343 // (850.0-370.0-274.0/2.0)
        //lidarTestPtr.reset();

        LidarRPLidarA2::Ptr lidarTestPtr = std::make_shared<LidarRPLidarA2>( QString("/dev/ttyAL6"), 256000, std::make_shared< ItemRegister >( hal->_motor5Value ));
        lidarTestPtr->LOG().enableDebug(false);

        lidarTestPtr->init();

        // SETUP ODO
        odometryPtr->setPosition(rpos);

        Recalage::Ptr recalage = std::make_shared<Recalage>();


        recalage->LOG().enableDebug(false);

        recalage->setCalibrationMode(true);
        recalage->setLidarPosition(lidarPos.x,lidarPos.y,lidarPos.theta);
        targetHz = 2.0;
        recalage->setTargetSpeedHz(targetHz);

        recalage->init(odometryPtr,lidarTestPtr);

        // Bordure setup
        recalage->borderListAdd(0,0   ,-850,0   ,850);
        recalage->borderListAdd(1,0   ,850 ,1500,850);
        recalage->borderListAdd(0,1500,850 ,1500,-850);
        recalage->borderListAdd(1,1500,-850,0   ,-850);

        i = 0;

        TrajectoryManagerHw _trajectoryManager( hal );
        _trajectoryManager.init();
        _trajectoryManager.enable();

        uint8_t state = 0;
        uint8_t tour_count = 3;

        // check first if init ok & target speed ok
        while (recalage->isRunning())
        {

            double hz;
            recalage->getSpeedHz(hz);
            RobotPos posOdo;
            RobotPos posLidar,errorLidar;
            recalage->getLatestCalibratedPos(posLidar);
            recalage->getAccumulatedError(errorLidar);
            posOdo = odometryPtr->getPosition();

            if (recalage->isInitDone())
            {
                tInfo(LOG) << "testRecalage: Pos Odo X/Y/T" << posOdo.x << posOdo.y << DEG(posOdo.theta);
                tInfo(LOG) << "testRecalage: Pos Ldr X/Y/T" << posLidar.x << posLidar.y << DEG(posLidar.theta);
                tInfo(LOG) << "testRecalage: Err Ldr X/Y/T" << errorLidar.x << errorLidar.y << DEG(errorLidar.theta);

                if (_trajectoryManager.isTrajReady())
                {
                    tInfo(LOG) << "testRecalage: isTrajReady";

                    switch (state)
                    {
                        case 0:
                            //_trajectoryManager.turnAAbs(0,true);
                            _trajectoryManager.moveForwardToXYAbs(0,600,-350);
                            state++;
                            tInfo(LOG) << "Go from " << state-1 << "to" << state;

                            break;
                        case 1:
                            _trajectoryManager.moveForwardToXYAbs(0,600,50);
                            //_trajectoryManager.turnAAbs(90,true);

                            state++;
                            tInfo(LOG) << "Go from " << state-1 << "to" << state;
                            break;
                        case 2:
                            _trajectoryManager.moveForwardToXYAbs(0,300,50);
                            //_trajectoryManager.turnAAbs(0,true);

                            state++;
                            tInfo(LOG) << "Go from " << state-1 << "to" << state;

                            break;
                        case 3:
                            _trajectoryManager.moveForwardToXYAbs(0,300,-350);
                            //_trajectoryManager.turnAAbs(90,true);

                            state++;
                            tInfo(LOG) << "Go from " << state-1 << "to" << state;

                            break;
                        case 4:
                            //_trajectoryManager.turnAAbs(0,false);
                            _trajectoryManager.turnAAbs(0,true);

                            state++;
                            tInfo(LOG) << "Go from " << state-1 << "to" << state;

                            break;
                        case 5:
                            if (tour_count--)
                            {
                                tInfo(LOG) << "Tour remaining:" << tour_count;
                                state = 0;
                            } else {
                                tInfo(LOG) << "Finished !";
                                _trajectoryManager.disable();
                            }

                            break;
                    }
                }




            }
            QThread::msleep(500);
        }

    } // END TEST 1


    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";
    tInfo(LOG) << "SUCCESS SUCCESS SUCCESS";

    QCoreApplication::quit();
    thread_logs.join();
}
