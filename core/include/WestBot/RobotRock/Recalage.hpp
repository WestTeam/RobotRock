// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_RECALAGE_HPP_
#define WESTBOT_ROBOTROCK_RECALAGE_HPP_

#include <memory>

#include <QMutex>

#include "Hal.hpp"
#include "ItemRegister.hpp"

namespace WestBot {
namespace RobotRock {

typedef struct
{
    double x;
    double y;
    double theta;
} RobotPos;

/*!
 * \brief The Recalage class allow to process lidar measurement
 *        to update the robot internal odometry.
 *
 *        This module is thread-safe because it can be called
 *        from multiple thread such as: Lidar and Strategy threads.
 */
class Recalage
{
public:
    using Ptr = std::shared_ptr< Recalage >;

    Recalage();
    ~Recalage();

    bool init( const Hal::Ptr& hal );

    void errorInit( double errX, double errY, double errTheta );

    void errorModify( double errX, double errY, double errTheta );

    bool calibrate(
        int len,
        const double* mesR,
        const double* mesTheta );

    RobotPos getPos(); // Unused for now
    RobotPos sendPos( const RobotPos& robotPos );

private:
    ItemRegister::Ptr _odoThetaReg;
    ItemRegister::Ptr _odoXReg;
    ItemRegister::Ptr _odoYReg;
    bool _attached;

    RobotPos error = { 0, 0, 0 };

    struct {
        bool dir;
        double	ax;
        double	ay;
        double	bx;
        double	by;
    } tableBorder[6] = {
        { 0,      0, -1500,      0,      1500 },
        { 1,      0,  1500,   2000,      1500 },
        { 0,   2000,  1500,   2000,     -1500 },
        { 1,   2000, -1500,      0,     -1500 },
        { 0, 360+22, -1500, 360+22, -1500+710 },
        { 0, 360+22,  1500, 360+22,  1500-710 },
    };

    int tableBorderNb = sizeof( tableBorder ) / sizeof( tableBorder[ 0 ] );

    QMutex* _lock;
};

}
}

#endif // WESTBOT_ROBOTROCK_RECALAGE_HPP_
