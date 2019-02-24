// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_LIDAR_HPP_
#define WESTBOT_ROBOTROCK_LIDAR_HPP_

#include <QThread>

#include <WestBot/RPLidar/RPLidar.hpp>

class QString;

namespace WestBot {
namespace RobotRock {

class Lidar : public QThread
{
public:
    Lidar( const QString& lidarTTY = "/dev/ttyUSB0" );

    bool init();

    void startScan();
    void stopScan();
    bool calibrate();

private:
    void run();

private:
    RPLidar::RPLidar _lidar;
};

}
}

#endif // WESTBOT_ROBOTROCK_LIDAR_HPP_
