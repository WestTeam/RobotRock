// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ODOMETRY_HPP_
#define WESTBOT_ROBOTROCK_ODOMETRY_HPP_

#include <memory>

#include <math.h>

#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * M_PI / 180.0)

namespace WestBot {
namespace RobotRock {

typedef struct
{
    double x; // mm
    double y; // mm
    double theta; // rad
} __attribute__( ( packed ) ) RobotPos;

bool operator==( RobotPos& lhs, RobotPos& rhs );

class Odometry
{
public:
    using Ptr = std::shared_ptr< Odometry >;

    virtual ~Odometry();

    virtual RobotPos getPosition() = 0;
    virtual void setPosition( RobotPos pos ) = 0;
    virtual void addError( RobotPos error ) = 0;
};

}
}

#endif // WESTBOT_ROBOTROCK_ODOMETRY_HPP_
