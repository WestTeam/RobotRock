// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_MONITORING_HPP_
#define WESTBOT_ROBOTROCK_MONITORING_HPP_

#include <memory>

#include <QThread>

#include "Common.hpp"
#include "Hal.hpp"
#include "ArmsManager.hpp"
#include "Nextion.hpp"
#include "Odometry.hpp"

namespace WestBot {
namespace RobotRock {

class Monitoring : public QThread
{
public:
    using Ptr = std::shared_ptr< Monitoring >;

    Monitoring(
        const Hal::Ptr& hal,
        const Odometry::Ptr& odometry,
        const ArmsManager::Ptr& armsManager );

    void setRefreshRate( int delayMs );

    void updateColor( Color color );

private:
    void run();

    void dump();

private:
    Hal::Ptr _hal;
    Odometry::Ptr _odo;
    ArmsManager::Ptr _armsManager;
    Nextion _screen;
    int _delayMs;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_MONITORING_HPP_
