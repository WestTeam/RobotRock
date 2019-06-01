// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_
#define WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_

#include <memory>
#include <QObject>

#include <WestBot/HumanAfterAll/Category.hpp>

#include "Hal.hpp"
#include "ItemRegister.hpp"
#include "Odometry.hpp"
#include "Lidar.hpp"
#include "LidarCircle.hpp"

namespace WestBot {
namespace RobotRock {

class OpponentDetection: public QThread
{
    Q_OBJECT
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.OpponentDetection");

    using Ptr = std::shared_ptr< OpponentDetection >;

    OpponentDetection();
    ~OpponentDetection();

    bool init( const Hal::Ptr& hal, const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar );

    bool isInitDone();

    void setLidarPosition(double x, double y, double theta0);

    void setTargetSpeedHz(double hz);
    bool getSpeedHz(double &hz);

    QList<LidarCircle::Obstacle> locate(
        LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
        uint32_t dataCount);

private:
    Hal::Ptr _hal;

    bool _attached = false;

    Odometry::Ptr _odometry;
    LidarBase::Ptr _lidar;

    std::atomic<bool> _initDone;
    std::atomic<bool> _finishing;

    std::atomic<double> _speedTargetHz;
    std::atomic<double> _currentSpeedHz;

    int _speedReductor;

    // position from robot center
    double _lidarPosX = 0.0;
    double _lidarPosY = 0.0;
    // theta0 offset to apply on lidar data to sync with robot own theta
    double _lidarTheta0 = 0.0;

    LidarData data[LIDAR_MAX_SCAN_POINTS];

    QMutex _lock;

    void emitObstacle(double x, double y);

signals:
    // Signals does not return something. Always void...
    void opponentDetected( double x, double y );

private:
    // thread run
    void run() override;
};

}
}

#endif // WESTBOT_ROBOTROCK_OPPONENTDETECTION_HPP_
