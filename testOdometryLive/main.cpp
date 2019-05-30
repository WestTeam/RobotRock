#include <QCoreApplication>
#include <QThread>
#include <QDate>
#include <random>
#include <QSerialPort>



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
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.testOdometryLive" )
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

    ///// GENERAL CONFIG /////

    RobotPos rpos; // real position

    Odometry::Ptr odometryPtr = std::make_shared< OdometryHw >( hal );
    //LidarTest::Ptr lidarTestPtr;// = std::make_shared< LidarTest >(odometryPtr);


    hal->_resetAll.write( 1 );

    hal->clearRegisters();

    hal->_resetAll.write( 0 );


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
    hal->_pidAngleKp.write( ( float ) 600000.0 );
    hal->_pidAngleKi.write( ( float ) 0.0 );
    hal->_pidAngleKd.write( ( float ) 0.0 );

    hal->_pidAngleSpeed.write( ( float ) 0.0001 );
    hal->_pidAngleAcceleration.write( ( float ) 0.00000002 );
    hal->_pidAngleSaturation.write( 25000 );

    hal->_pidAngleTarget.write( hal->_pidAnglePosition.read< float >() );
    hal->_pidAngleEnable.write( 1 );




    // TEST 5: Test InitDone with RPLIDAR
    {
        //tId = 5;

        rpos = {.x = 0.0, .y = 0.0, .theta = RAD(0)};

        LidarRPLidarA2::Ptr lidarTestPtr = std::make_shared<LidarRPLidarA2>( QString("/dev/ttyAL6"), 256000, std::make_shared< ItemRegister >( hal->_pwmCustom0Value ));
        lidarTestPtr->LOG().enableDebug(false);

        lidarTestPtr->init();

        // SETUP ODO
        //odometryPtr->setPosition(rpos);
        //i = 0;

        TrajectoryManagerHw _trajectoryManager( hal );
        _trajectoryManager.init();
        _trajectoryManager.enable();

        uint8_t state = 0;


        QSerialPort* serial_odo = new QSerialPort("/dev/ttyAL5");
        tInfo( LOG ) << "1";
        serial_odo->setBaudRate(115200);
        tInfo( LOG ) << "2";
        serial_odo->open(QIODevice::ReadOnly);


        QString file = "ododata.txt";
        QFile f("/home/" + file);
        f.open(QIODevice::WriteOnly);

        lidarTestPtr->startMotor(30.0);
        lidarTestPtr->startScan();


        int j;
        bool doing = true;
        while (doing)
        {
            LidarData data[LIDAR_MAX_SCAN_POINTS];
            uint32_t dataCount;

            bool ok = lidarTestPtr->get360ScanData(data,dataCount);

            if (ok)
            {
                QString file = "setup0-lidardata" + QString::number(j) + ".txt";
                j++;
                QFile f("/home/" + file);
                f.open(QIODevice::WriteOnly);
                for (int i = 0; i< dataCount; i++)
                {
                    f.write(QString("%1;%2;%3;%4;%5;%6\n").arg(data[i].r).arg(data[i].theta).arg(data[i].quality).arg(data[i].pos.x).arg(data[i].pos.y).arg(data[i].pos.theta).toStdString().c_str());
                }
                f.close();
            }
            serial_odo->waitForReadyRead(5);
            QByteArray array = serial_odo->readAll();

            if (array.size() > 0)
            {

                f.write(array);
                f.flush();
            }

            if (_trajectoryManager.isTrajReady())
            {
                QThread::msleep(2000);

                tInfo(LOG) << "testRecalage: isTrajReady";

                float x,y;

                x = (1274.8-52.0-100.0)*2.0;
                y = 100;

                switch (state)
                {
                    case 0:
                        //_trajectoryManager.turnAAbs(0,true);
                        _trajectoryManager.moveForwardToXYAbs(0,x,0,true);
                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 1:

                        _trajectoryManager.moveForwardToXYAbs(0,x,y,true);
                        //_trajectoryManager.turnAAbs(90,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;
                        break;
                    case 2:
                        _trajectoryManager.moveForwardToXYAbs(0,0,y,true);
                        //_trajectoryManager.turnAAbs(0,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 3:
                        _trajectoryManager.moveForwardToXYAbs(0,0,0,true);
                        //_trajectoryManager.turnAAbs(90,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 4:
                    _trajectoryManager.moveForwardToXYAbs(0,x,0,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;



                    case 5:
                        //_trajectoryManager.turnAAbs(0,true);
                        _trajectoryManager.moveForwardToXYAbs(0,0,0,true);
                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 6:

                        _trajectoryManager.moveForwardToXYAbs(0,0,y,true);
                        //_trajectoryManager.turnAAbs(90,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;
                        break;
                    case 7:
                        _trajectoryManager.moveForwardToXYAbs(0,x,y,true);
                        //_trajectoryManager.turnAAbs(0,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 8:
                        _trajectoryManager.moveForwardToXYAbs(0,x,0,true);
                        //_trajectoryManager.turnAAbs(90,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;

                        break;
                    case 9:
                        _trajectoryManager.moveForwardToXYAbs(0,0,0,true);

                        state++;
                        tInfo(LOG) << "Go from " << state-1 << "to" << state;



                        break;

                    case 10:
                        QThread::msleep(2000);
                        state++;
                        break;
                case 11:
                    doing = false;
                    break;
                    }

            }
            QThread::msleep(500);
        }
        f.close();

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
