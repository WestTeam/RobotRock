// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_PUCKDETECTION_HPP_
#define WESTBOT_ROBOTROCK_PUCKDETECTION_HPP_

#include <WestBot/HumanAfterAll/Category.hpp>

#include <memory>
#include <atomic>

#include <QThread>
#include <QMutex>

#include "Hal.hpp"
#include "ItemRegister.hpp"
#include "Odometry.hpp"
#include "Lidar.hpp"
#include "LidarCircle.hpp"



namespace WestBot {
namespace RobotRock {

struct PuckLidarPos {
    float			X = 0;
    float			Y = 0;
    float			Q = 0;
};



/*!
 * \brief The PuckDetection class gives absolute position of pucks
 *
 *        This module is thread-safe because it can be called
 *        from multiple thread such as: Strategy threads.
 */
class PuckDetection: public QThread
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.PuckDetection" );

    using Ptr = std::shared_ptr< PuckDetection >;

    PuckDetection();
    ~PuckDetection();

    bool init( const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar );

    bool isInitDone();

    void setLidarPosition(double x, double y, double theta0);

    void setTargetSpeedHz(double hz);
    bool getSpeedHz(double &hz);


    QList<PuckLidarPos> locate(
        LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
        uint32_t dataCount);

    QList<PuckLidarPos> getPucks();

private:

    QList<PuckLidarPos> _pucksList;

    LidarCircle _lidarCircle;

    bool _attached = false;

    Odometry::Ptr _odometry;
    LidarBase::Ptr _lidar;

    std::atomic<bool> _initDone;
    std::atomic<bool> _continuousMode ;
    std::atomic<bool> _trigger;
    std::atomic<bool> _finishing;


    std::atomic<double> _speedTargetHz;
    std::atomic<double> _currentSpeedHz;
    RobotPos _latestPos = { 0, 0, 0 };
    // position from robot center
    double _lidarPosX = 0.0;
    double _lidarPosY = 0.0;
    // theta0 offset to apply on lidar data to sync with robot own theta
    double _lidarTheta0 = 0.0;
    // theta between robot center and lidar center
    double _lidarTheta = 0.0;
    // distance between robot center and lidar center
    double _lidarR = 0.0;

    QMutex _lock;

private:
    LidarData data[LIDAR_MAX_SCAN_POINTS];


    // thread run
    void run() override;
};

}
}

#endif // WESTBOT_ROBOTROCK_PUCKDETECTION_HPP_
