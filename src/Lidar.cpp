// Copyright (c) 2018 All Rights Reserved WestBot

#include <cstdio>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Lidar.hpp>

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Lidar" )

    void plot_histogram( RPLidar::measurementNode_t* nodes, size_t count )
    {
        const int BARCOUNT =  75;
        const int MAXBARHEIGHT = 20;
        const float ANGLESCALE = 360.0f/BARCOUNT;

        qDebug() << "Plot histogramme data";

        float histogram[BARCOUNT];
        for (int pos = 0; pos < _countof(histogram); ++pos) {
            histogram[pos] = 0.0f;
        }

        float max_val = 0;
        for (int pos =0 ; pos < (int)count; ++pos) {
            int int_deg = (int)((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f/ANGLESCALE);
            if (int_deg >= BARCOUNT) int_deg = 0;
            float cachedd = histogram[int_deg];
            if (cachedd == 0.0f ) {
                cachedd = nodes[pos].distance_q2/4.0f;
            } else {
                cachedd = (nodes[pos].distance_q2/4.0f + cachedd)/2.0f;
            }

            if (cachedd > max_val) max_val = cachedd;
            histogram[int_deg] = cachedd;
        }

        for (int height = 0; height < MAXBARHEIGHT; ++height) {
            float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
            for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
                if (histogram[xpos] >= threshold_h) {
                    putc('*', stdout);
                }else {
                    putc(' ', stdout);
                }
            }
            printf("\n");
        }
        for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
            putc('-', stdout);
        }
        printf("\n");
    }

    bool capture_and_display( RPLidar::RPLidar& lidar )
    {
        bool ans = false;

        RPLidar::measurementNode_t nodes[ 8192 ];
        size_t   count = _countof(nodes);

        qDebug() << "waiting for data...\n";

        // fetech extactly one 0-360 degrees' scan
        ans = lidar.grabScanData(nodes, count);
        if( ans )
        {
            qDebug() << "Grabing scan data: OK";
            lidar.ascendScanData( nodes, count );
            plot_histogram(nodes, count);
        }
        else
        {
            qDebug() << "Error";
        }

        return ans;
    }
}

Lidar::Lidar( Recalage& recalage )
    : _lidar( "/dev/ttyUSB0" )
    , _recalage( recalage )
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

    _lidar.startMotor();

    QThread::msleep( 200 );

    _lidar.startScan();

    RPLidar::measurementNode_t nodes[ 8192 ];
    size_t count = _countof(nodes);

    double mesR[ ( int ) count ];
    double mesTheta[ ( int ) count ];

    tDebug( LOG ) << "waiting for data...";

    // Fetch exactly one 0-360 degrees' scan
    if( _lidar.grabScanData(nodes, count) )
    {
        tDebug( LOG ) << "Grabing scan data: OK";

        if( ! _lidar.ascendScanData( nodes, count ) )
        {
            return false;
        }

        for( int pos = 0; pos < ( int ) count; ++pos )
        {
            mesR[ pos ] = static_cast< double >( nodes[ pos ].distance_q2 );
            mesTheta[ pos ] = static_cast< double >( ( nodes[ pos ].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT ) / 64.0f );
        }

        _recalage.calibrate( count, mesR, mesTheta );
    }
    else
    {
        tFatal( LOG ) << "Lidar scan and calibration failure";
    }

    stop();

    return true;
}

void Lidar::stop()
{
    _lidar.stopScan();
    _lidar.stopMotor();
}

void Lidar::run()
{
    _lidar.startMotor();

    QThread::msleep( 200 );

    _lidar.startScan();

    while( 1 )
    {
        capture_and_display( _lidar );
        QThread::msleep( 1000 );
    }
}
