// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QString>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Lidar.hpp>

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT 2


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define SIMU


using namespace WestBot;
using namespace WestBot::RobotRock;



LidarRPLidarA2::LidarRPLidarA2( const QString& lidarTTY, const uint32_t baudrate, ItemRegister::Ptr Pwm)
    : _lidar( lidarTTY, baudrate )
{

    _pwm = Pwm;
    _minQuality = 0;
}

LidarRPLidarA2::~LidarRPLidarA2()
{
    tDebug( LOG ) << "Lidar: module destruction";

    stopScan();
    stopMotor();
    _lidar.disconnect();
}



bool LidarRPLidarA2::init()
{
    if (! _lidar.connect() )
    {
       tWarning( LOG ) << "Lidar: Cannot connect to RPLidar";
       return false;
      // throw std::exception();
    }
    tDebug( LOG ) << "Lidar: RPLidar connected";

    QString lidarInfo = _lidar.getDeviceInfo();
    tDebug( LOG ).noquote() << lidarInfo;

    // check RPLidar health...
    if( ! _lidar.checkHealth() )
    {
        tWarning( LOG ) << "Lidar: RPLidar not healthy";
        return false;

        //throw std::exception();
    }

    tDebug( LOG ) << "Lidar: module initialized !";

    return true;
}


void LidarRPLidarA2::setMinimumQuality(uint8_t minQuality)
{
    _minQuality = minQuality;
}


QString LidarRPLidarA2::info()
{
     return _lidar.getDeviceInfo();
}

bool LidarRPLidarA2::health()
{
    return _lidar.checkHealth();
}


void LidarRPLidarA2::startMotor(float percentage)
{
    // if null, we send command to the underlying library
    if (_pwm == nullptr)
    {
        uint16_t pwmValue = (uint16_t)((float)1024*percentage/100.0);
        _lidar.setMotorPwm( pwmValue );
    } else { // otherwise we set our own pwm
        int16_t pwmValue = (int16_t)((float)32767*percentage/100.0);
        _pwm->write(pwmValue);
    }
}

void LidarRPLidarA2::stopMotor()
{
    startMotor(0.0);
}

void LidarRPLidarA2::startScan()
{
    _lidar.startScanNormalRobotPos(false);
}

void LidarRPLidarA2::stopScan()
{
    _lidar.stopScan();
}

bool LidarRPLidarA2::get360ScanData(LidarData (&data)[LIDAR_MAX_SCAN_POINTS], uint32_t &count)
{
    QMutexLocker locker( & _lock );

    size_t scan_count = _countof(nodes);

    count = 0;

    //double mesR[ ( int ) count ];
    //double mesTheta[ ( int ) count ];
    //int syncBit[ ( int ) count ];

    // Fetch exactly one 0-360 degrees' scan
    if( _lidar.grabScanData(nodes, scan_count, 2000) )
    {
        //tDebug( LOG ) << "Grabing scan data: OK" << scan_count;

        /*
        if( ! _lidar.ascendScanData( nodes, scan_count ) )
        {
            return false;
        }*/

        unsigned int pos = 0;

        for( pos = 0; pos < scan_count; ++pos )
        {
            uint8_t quality = nodes[ pos ].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

            if (quality >= _minQuality)
            {
                data[ count ].quality = quality;
                data[ count ].r =
                   static_cast< double >( nodes[ pos ].distance_q2 / 4.0f );
                data[ count ].theta = RAD(
                   static_cast< double >(
                       ( nodes[ pos ].angle_q6_checkbit >>
                         RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT ) / 64.0f ));
                data[ count ].pos.x = (double)nodes[pos].pos_x;
                data[ count ].pos.y = (double)nodes[pos].pos_y;
                data[ count ].pos.theta = RAD((double)nodes[pos].pos_teta/100.0);
                data[ count ].ts = ((uint32_t)((uint16_t)nodes[pos].pos_y)) << 16 | (uint32_t)((uint16_t)nodes[pos].pos_x);

                count++;
            }
        }
    }
    else
    {
        stopScan();
        startScan();
        return false;
    }

    return true;
}
/*
bool Lidar::grabScanData()
{
    RPLidar::measurementNode_t nodes[ 8192 ];
    size_t count = _countof( nodes );

    tDebug( LOG ) << "Waiting for data...";

    // Fetch exactly one 0-360 degrees' scan
    if( _lidar.grabScanData( nodes, count ) )
    {
        tDebug( LOG ) << "Grabing scan data: OK";
        return true;
    }

    return false;
}

bool Lidar::ascendScanData()
{
    RPLidar::measurementNode_t nodes[ 8192 ];
    size_t count = _countof( nodes );

    double mesR[ ( int ) count ];
    double mesTheta[ ( int ) count ];

    tDebug( LOG ) << "Waiting for data...";

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
            mesR[ pos ] =
               static_cast< double >( nodes[ pos ].distance_q2 / 4.0f );
            mesTheta[ pos ] =
               static_cast< double >(
                   ( nodes[ pos ].angle_q6_checkbit >>
                     RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT ) / 64.0f );

            tDebug( LOG )
                << "Mes @ pos:" << pos << " R = " << mesR[ pos ]
                << "Theta = " << mesTheta[ pos ] << "x" << nodes[ pos ].pos_x << "y" << nodes[ pos ].pos_y << "teta" << nodes[ pos ].pos_teta;
        }

        return true;
    }

    return false;
}

void Lidar::run()
{
    startScan();

    while( 1 )
    {
        calibrate();

        QThread::msleep( 1000 );
    }
}*/
