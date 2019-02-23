// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_LIDAR_HPP_
#define WESTBOT_ROBOTROCK_LIDAR_HPP_

#include <QThread>

#include <WestBot/RPLidar/RPLidar.hpp>

#include "Recalage.hpp"

namespace WestBot {
namespace RobotRock {

class Lidar : public QThread
{
public:
    Lidar( const Recalage::Ptr& recalage );

    bool init();

    void startScan();
    void stopScan();
    bool calibrate();

private:
    void run();

private:
    RPLidar::RPLidar _lidar;
    Recalage::Ptr _recalage;
};

}
}

#endif // WESTBOT_ROBOTROCK_LIDAR_HPP_
