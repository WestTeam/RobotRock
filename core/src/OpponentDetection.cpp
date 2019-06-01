// Copyright (c) 2018 All Rights Reserved WestBot

#include <WestBot/HumanAfterAll/Category.hpp>


namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.OpponentDetection" )
}

// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <iostream>
#include <math.h>

#include <QMutexLocker>
#include <QDate>
#include <QList>

#include <WestBot/RobotRock/Hal.hpp>
#include <WestBot/RobotRock/Odometry.hpp>
#include <WestBot/RobotRock/OpponentDetection.hpp>



using namespace WestBot;
using namespace WestBot::RobotRock;


OpponentDetection::OpponentDetection()
    : _odometry( nullptr )
    , _lidar( nullptr )
{
    tDebug( LOG ) << "Opponent Detection constructor engaged";

    _initDone = false;
    _finishing = false;

    _speedTargetHz = 1.0;
    _currentSpeedHz = 0.0;

    _speedReductor = 0;
}


OpponentDetection::~OpponentDetection()
{
    tDebug( LOG ) << "OpponentDetection destructor called...";

    if( _attached )
    {
        _finishing = true;

        this->QThread::wait(2000);

        _lidar->stopScan();
        _lidar->stopMotor();
    }
    tDebug( LOG ) << "OpponentDetection thread stopped";
}

bool OpponentDetection::init( const Hal::Ptr& hal, const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar )
{
    if( ! _attached )
    {
        _hal = hal;
        _odometry = odometry;
        _lidar = lidar;

        // launch data thread
        this->start();
    }

    _attached = true;

    tInfo( LOG ) << "OpponentDetection initialized";

    return true;
}

bool OpponentDetection::isInitDone()
{
    return _initDone;
}


void OpponentDetection::setLidarPosition(double x, double y, double theta0)
{
    _lidarPosX = x;
    _lidarPosY = y;
    _lidarTheta0 = theta0;

}

void OpponentDetection::setTargetSpeedHz(double hz)
{
    _speedTargetHz = hz;
    _initDone = false;
}

bool OpponentDetection::getSpeedHz(double &hz)
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



QList<LidarCircle::Obstacle> OpponentDetection::locate(
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

    LidarCircle _lidarCircle(100.0,_lidarPosX,_lidarPosY,-1.0);

    QList<LidarCircle::Obstacle> ret;
    QList<LidarCircle::Obstacle> list = _lidarCircle.compute(a,d);
    QList<LidarCircle::Obstacle> list_rel = list;
    QList<LidarCircle::Obstacle> list_abs;

    RobotPos robotPos = data[0].pos;

    _lidarCircle.transformToRobot(list_rel);
    list_abs = list_rel;
    _lidarCircle.transformToAbs(list_abs,robotPos.x,robotPos.y,robotPos.theta);
    _lidarCircle.associate(list_abs);

    for (int i = 0; i < list.size(); i++)
    {
        float dist = hypot(list_rel[i].X,list_rel[i].Y);

        if (list_rel[i].Q >= 0.3 && dist <= 1000.0 && list_abs[i].type == LidarCircle::Obstacle::Type::ROBOT)
        {

            ret << list_abs[i];

            //tInfo( LOG ) << "Robot Detection:Abs:" << dist << list_abs[i].X << list_abs[i].Y << list_abs[i].Q;
            //tInfo( LOG ) << "Robot Detection:Rel:" << dist << list_rel[i].X << list_rel[i].Y << list_rel[i].Q;
        }
    }

    return ret;
}


void OpponentDetection::run()
{
    tDebug( LOG ) << "OpponentDetection: Run";

    #define PERCENTAGE_START 25.0
    #define PERCENTAGE_STEP 2.5
    #define RETRY_COUNT 5
    #define STABLE_COUNT ((uint32_t)(1.0/(1.0/_speedTargetHz)))

    _lidar->setMinimumQuality(0);
    float mPercentage = PERCENTAGE_START;
    _lidar->startMotor(mPercentage);
    this->QThread::msleep(2000);
    _lidar->startScan();


    uint64_t last_scan_ok = 0;
    uint8_t retry_count = 0;
    uint8_t consecutive_speed_ok = 0;
    uint8_t consecutive_speed_ko = 0;
    bool ok;
    unsigned int dropScanCount = 0;
    uint32_t dataCount;

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
                    _lidar->stopScan();
                    _lidar->startScan();

                    last_scan_ok = 0;
                }
            }
        }
        else
        {
            ok = _lidar->get360ScanData(data,dataCount);

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

                // first calibration in order to correct theta first
                if (ok)
                {
                    QList<LidarCircle::Obstacle> list = locate(data,dataCount);

                    uint8_t speed_reductor = 0;

                    for (int i = 0; i < list.length(); i++)
                    {
                        float dist = hypot(list[i].X-data[0].pos.x, list[i].Y-data[0].pos.y);

                        if (dist < 1000)
                            speed_reductor = 100-50;
                        if (dist < 800)
                            speed_reductor = 100-30;
                        if (dist < 650)
                            speed_reductor = 100-0;
                        if (dist < 400)
                            speed_reductor = 100-0;
                    }

                    _hal->_pidDistanceSpeedReductor.write(speed_reductor);
                    _hal->_pidAngleSpeedReductor.write(speed_reductor);
                    if (speed_reductor != 0 || _speedReductor != speed_reductor)
                        tDebug( LOG ) << "Recalage: Speed Reductor to " << speed_reductor << "%";

                    _speedReductor = speed_reductor;
                }
            }
        }
    }
}

void OpponentDetection::emitObstacle(double x, double y)
{
    emit opponentDetected(x,y);

}
