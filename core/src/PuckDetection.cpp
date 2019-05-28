// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <iostream>
#include <math.h>


#include <QMutexLocker>
#include <QDate>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/PuckDetection.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;


PuckDetection::PuckDetection()
    : _odometry( nullptr )
    , _lidarCircle(75)
    , _lidar( nullptr )
{
    tDebug( LOG ) << "PuckDetection constructor engaged";

    _initDone = false;
    _continuousMode = true;
    _trigger = false;
    _finishing = false;

    _lidarPosX = 0.0;
    _lidarPosY = 0.0;
    _lidarTheta0 = 0.0;


    _speedTargetHz = 1.0;
    _currentSpeedHz = 0.0;
}


PuckDetection::~PuckDetection()
{
    tDebug( LOG ) << "PuckDetection destructor called...";

    if( _attached )
    {
        _finishing = true;

        this->QThread::wait(2000);

        _lidar->stopScan();
        _lidar->stopMotor();
    }
    tDebug( LOG ) << "PuckDetection thread stopped";

}

bool PuckDetection::init( const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar )
{
    if( ! _attached )
    {
        _odometry = odometry;
        _lidar = lidar;

        // launch data thread
        this->start();
    }

    _attached = true;

    tInfo( LOG ) << "PuckDetection initialized";

    return true;
}

bool PuckDetection::isInitDone()
{
    return _initDone;
}

void PuckDetection::setLidarPosition(double x, double y, double theta0)
{
    _lidarPosX = x;
    _lidarPosY = y;
    _lidarTheta0 = theta0;

    _lidarTheta = atan2( _lidarPosY, _lidarPosX );
    _lidarR = sqrt( _lidarPosX * _lidarPosX + _lidarPosY * _lidarPosY );

}

void PuckDetection::setTargetSpeedHz(double hz)
{
    _speedTargetHz = hz;
    _initDone = false;
}

bool PuckDetection::getSpeedHz(double &hz)
{
    if (_initDone == true && _currentSpeedHz != 0.0)
    {
        hz = _currentSpeedHz;
    } else {
        hz = 0;
        return false;
    }
    return true;
}


QList<PuckLidarPos> PuckDetection::locate(
    LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
    uint32_t dataCount)
{
    QVector<float> d;
    QVector<float> a;
    QVector<float> q;

    for (int i = 0; i < dataCount; i++)
    {
        d << data[i].r;
        a << data[i].theta;
        q << data[i].quality;
    }

    QList<LidarCircle::Obstacle> list = _lidarCircle.compute(a,d);

    QList<PuckLidarPos> ret;

    RobotPos robotPos = data[0].pos;

    for ( QList<LidarCircle::Obstacle>::iterator it = list.begin(); it != list.end(); ++it){
        PuckLidarPos pos;

        pos.Q = it->Q;
        pos.X = robotPos.x+(_lidarPosX+it->X)*cos(robotPos.theta)+(_lidarPosY+it->Y)*sin(robotPos.theta);
        pos.Y = robotPos.y+(_lidarPosY+it->Y)*cos(robotPos.theta)-(_lidarPosX+it->X)*sin(robotPos.theta);

        if (pos.Q >= 1.0 && hypot(pos.X,pos.Y) <= 500.0)
        {
            ret << pos;

            tInfo( LOG ) << "PuckDetection: Abs:" << pos.X << pos.Y << pos.Q;
            tInfo( LOG ) << "PuckDetection: Rel:" << it->X << it->Y;


        }
    }

    return ret;
}

void PuckDetection::run()
{
    tDebug( LOG ) << "PuckDetection: Run";

#define PERCENTAGE_START 30
#define PERCENTAGE_STEP 2.5
#define RETRY_COUNT 5
#define STABLE_COUNT ((uint32_t)(1.0/(1.0/_speedTargetHz)))

    _lidar->setMinimumQuality(0);
    float mPercentage = PERCENTAGE_START;
    _lidar->startMotor(mPercentage);
    this->QThread::msleep(2000);
    _lidar->startScan();

    LidarData data[LIDAR_MAX_SCAN_POINTS];
    uint32_t dataCount;
    RobotPos currentError;
    RobotPos absPos;

    uint64_t last_scan_ok = 0;
    uint8_t retry_count = 0;
    uint8_t consecutive_speed_ok = 0;
    uint8_t consecutive_speed_ko = 0;
    bool ok;
    unsigned int dropScanCount = 0;

    while (!_finishing)
    {
        if( ! _initDone )
        {
                ok = _lidar->get360ScanData(data,dataCount);
                if (ok && dataCount != 0)
                {
                    uint64_t now = QDateTime::currentMSecsSinceEpoch();

                    if (last_scan_ok != 0)
                    {
                        uint64_t period = now-last_scan_ok;

                        last_scan_ok = now;

                        double freq = 1.0/((double)(period)/1000.0);
                        _currentSpeedHz = freq;
                        tDebug( LOG ) << "Recalage: freq" << freq << period << consecutive_speed_ko;

                        if (freq <= _speedTargetHz)
                        {
                            consecutive_speed_ok = 0;
                            consecutive_speed_ko++;
                            if (consecutive_speed_ko==STABLE_COUNT)
                            {
                                mPercentage+=PERCENTAGE_STEP;
                                if (mPercentage >= 100.0)
                                    mPercentage = 100.0;

                                _lidar->startMotor(mPercentage);
                                //this->QThread::msleep(2000);
                                last_scan_ok = 0;
                            }
                        } else {
                            consecutive_speed_ok++;
                            if (consecutive_speed_ok==STABLE_COUNT)
                            {
                                _initDone = true;
                            }
                        }
                    } else {
                        consecutive_speed_ok = 0;
                        consecutive_speed_ko = 0;
                        last_scan_ok = now;
                    }

                    retry_count = 0;

                } else {
                    retry_count++;
                    if (retry_count == RETRY_COUNT)
                    {
                        consecutive_speed_ko = 0;
                        consecutive_speed_ok = 0;
                        retry_count = 0;

                        mPercentage+=PERCENTAGE_STEP;
                        if (mPercentage >= 100.0)
                            mPercentage = 100.0;

                        _lidar->startMotor(mPercentage);
                        //this->QThread::msleep(1000);
                        _lidar->stopScan();
                        _lidar->startScan();

                        last_scan_ok = 0;
                    }
                }
            }
        else
        {
                ok = _lidar->get360ScanData(data,dataCount);

                if (ok && dataCount < 50)
                {
                    _lidar->stopScan();
                    _lidar->startScan();
                    ok = false;
                    tInfo(LOG) << "fail" << ok << dataCount;

                } else if (!ok)
                {
                    _lidar->stopScan();
                    _lidar->startScan();
                }

                if (ok && dataCount != 0)
                {
                    uint64_t now = QDateTime::currentMSecsSinceEpoch();

                    uint64_t period = now-last_scan_ok;
                    last_scan_ok = now;

                    if (last_scan_ok != 0)
                    {
                        double freq = 1.0/((double)(period)/1000.0);
                        _currentSpeedHz = freq;
                    }

                    // if != 0 we continue (ie we do not process the data)
                    if (dropScanCount)
                    {
                        dropScanCount--;
                        continue;
                    }

                    bool ok;
                    // we check if the first pos is the same as last pos
                    ok = (data[0].pos == data[dataCount-1].pos);

                    tInfo(LOG) << ok << dataCount;

                    // first calibration in order to correct theta first
                    if (ok)
                    {
                        // second one to have correction on X/Y
                        QList<PuckLidarPos> list = locate(data,dataCount);


                        if (list.size() > 0)
                        {
                            tDebug( LOG ) << "Found pucks count: " << list.size();
                            // we lock the update of internal data that gets outputed outside
                            QMutexLocker locker( &_lock );

                            QThread::msleep(2000);

                            _pucksList = list;


                        }
                    }
                }
            }
        }

    }
