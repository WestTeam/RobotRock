// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ODOMETRY_HPP_
#define WESTBOT_ROBOTROCK_ODOMETRY_HPP_

#include <QDebug>

#include <math.h>
#include "Hal.hpp"
#include <QMutex>

#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * M_PI / 180.0)

namespace WestBot {
namespace RobotRock {

typedef struct
{
    double x; // mm
    double y; // mm
    double theta; // rad
} RobotPos;

bool operator==(RobotPos& lhs, RobotPos& rhs);


class Odometry
{
public:
    using Ptr = std::shared_ptr< Odometry >;

    Odometry(const Hal::Ptr& hal);

    RobotPos getPosition();
    void setPosition(RobotPos pos);
    void addError(RobotPos error);

private:
    Hal::Ptr _hal;
    RobotPos _posLatest;
    // for pos reading
    uint8_t _rd_pos_id;
    uint8_t _rd_pos_valid;
    // for pos writing
    uint8_t _wr_pos_id;
    uint8_t _wr_pos_valid;


    QMutex _lock;
};

}
}

#endif // WESTBOT_ROBOTROCK_ODOMETRY_HPP_
