// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_MONITORING_HPP_
#define WESTBOT_ROBOTROCK_MONITORING_HPP_

#include <QThread>

#include "Common.hpp"
#include "Hal.hpp"
#include "Nextion.hpp"
#include "Odometry.hpp"

namespace WestBot {
namespace RobotRock {

class Monitoring : public QThread
{
public:
    Monitoring( const Hal::Ptr& hal, const Odometry::Ptr& odometry );

    void setRefreshRate( int delayMs );

    void updateColor( Color color );

private:
    void run();

    void dump();

private:
    Hal::Ptr _hal;
    Odometry::Ptr _odo;
    Nextion _screen;
    int _delayMs;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_MONITORING_HPP_