// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QString>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Lidar.hpp>

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define SIMU

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Lidar" )
}

Lidar::Lidar( const QString& lidarTTY )
    : _lidar( lidarTTY )
{
}

bool Lidar::init()
{
    if( ! _lidar.connect() )
    {
       tWarning( LOG ) << "Cannot connect to RPLidar";
       return false;
    }

    QString lidarInfo = _lidar.getDeviceInfo();
    tDebug( LOG ).noquote() << lidarInfo;

    // check RPLidar health...
    if( ! _lidar.checkHealth() )
    {
        tWarning( LOG ) << "RPLidar not healthy";
        return false;
    }

    tInfo( LOG ) << "Lidar module initialized";

    startScan();

    if( ! calibrate() )
    {
        tWarning( LOG ) << "Lidar scan and first calibration failed";
        stopScan();
        return false;
    }

    stopScan();

    return true;
}

void Lidar::startScan()
{
    _lidar.setMotorPwm( 660 ); // default
    _lidar.startMotor();
    _lidar.startScan();
}

void Lidar::stopScan()
{
    _lidar.stopScan();
    _lidar.stopMotor();
}

bool Lidar::calibrate()
{
    tInfo( LOG ) << "Lidar calibration started";

    RPLidar::measurementNode_t nodes[ 8192 ];
    size_t count = _countof(nodes);

    double mesR[ ( int ) count ];
    double mesTheta[ ( int ) count ];
    int syncBit[ ( int ) count ];

    tDebug( LOG ) << "Waiting for data...";

    // Fetch exactly one 0-360 degrees' scan
    if( _lidar.grabScanData(nodes, count) )
    {
        tDebug( LOG ) << "Grabing scan data: OK";

        if( ! _lidar.ascendScanData( nodes, count ) )
        {
            return false;
        }

        int pos = 0;
        int newPos = 0;

        for( pos = 0; pos < ( int ) count; ++pos )
        {
            mesR[ pos ] =
               static_cast< double >( nodes[ pos ].distance_q2 / 4.0f );
            mesTheta[ pos ] =
               static_cast< double >(
                   ( nodes[ pos ].angle_q6_checkbit >>
                     RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT ) / 64.0f );
        }
    }
    else
    {
        return false;
    }

    tInfo( LOG ) << "Lidar calibration succeeded";

    return true;
}

void Lidar::run()
{
    startScan();

    while( 1 )
    {
        calibrate();

        QThread::msleep( 1000 );
    }
}
