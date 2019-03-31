// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_RECALAGE_HPP_
#define WESTBOT_ROBOTROCK_RECALAGE_HPP_

#include <WestBot/HumanAfterAll/Category.hpp>

#include <memory>
#include <atomic>

#include <QThread>
#include <QMutex>

#include "Hal.hpp"
#include "ItemRegister.hpp"
#include "Odometry.hpp"
#include "Lidar.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief The Recalage class allow to process lidar measurement
 *        to update the robot internal odometry.
 *
 *        This module is thread-safe because it can be called
 *        from multiple thread such as: Lidar and Strategy threads.
 */
class Recalage: public QThread
{
public:
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.Recalage" );

    using Ptr = std::shared_ptr< Recalage >;

    Recalage();
    ~Recalage();

    bool init( const Odometry::Ptr& odometry, const LidarBase::Ptr& lidar );

    bool isInitDone();

    void borderListClear();
    bool borderListAdd( uint8_t dir,double ax, double ay, double bx, double by );

    void setLidarPosition(double x, double y, double theta0);

    void setTargetSpeedHz(double hz);
    bool getSpeedHz(double &hz);

    void setCalibrationMode(bool continiuous = false);

    bool triggerCalibration(bool blocking);

    void getAccumulatedError(RobotPos &pos);
    void getLatestCalibratedPos(RobotPos &pos);

    //void errorInit( double errX, double errY, double errTheta );

    //void errorModify( double errX, double errY, double errTheta );

    bool calibrate(
        LidarData (&data)[LIDAR_MAX_SCAN_POINTS],
        uint32_t dataCount,
        RobotPos &currentError,
        RobotPos &absPos,
        double thetaBias);

    //RobotPos getPos(); // Unused for now
    //RobotPos sendPos( const RobotPos& robotPos );

private:

    bool _attached = false;

    Odometry::Ptr _odometry;
    LidarBase::Ptr _lidar;

    std::atomic<bool> _initDone;
    std::atomic<bool> _continuousMode ;
    std::atomic<bool> _trigger;
    std::atomic<bool> _finishing;


    std::atomic<double> _speedTargetHz;
    std::atomic<double> _currentSpeedHz;
    RobotPos _error = { 0, 0, 0 };
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



    #define BORDER_COUNT_MAX 32

    struct {
        bool dir;
        double	ax;
        double	ay;
        double	bx;
        double	by;
    } _tableBorder[BORDER_COUNT_MAX];

    /*= {
        { 0,      0, -1500,      0,      1500 },
        { 1,      0,  1500,   2000,      1500 },
        { 0,   2000,  1500,   2000,     -1500 },
        { 1,   2000, -1500,      0,     -1500 },
        { 0, 360+22, -1500, 360+22, -1500+710 },
        { 0, 360+22,  1500, 360+22,  1500-710 },
    };*/

    int _tableBorderNb = 0;//sizeof( tableBorder ) / sizeof( tableBorder[ 0 ] );

    QMutex _lock;

private:

    // thread run
    void run() override;
};

}
}

#endif // WESTBOT_ROBOTROCK_RECALAGE_HPP_
