// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_LIDAR_HPP_
#define WESTBOT_ROBOTROCK_LIDAR_HPP_

#include <QThread>

#include <WestBot/RPLidar/RPLidar.hpp>

namespace WestBot {
namespace RobotRock {

class Lidar : public QThread
{
public:
    Lidar();

    bool init();

    void stop();

private:
    void run();

private:
    RPLidar::RPLidar _lidar;
};

}
}

#endif // WESTBOT_ROBOTROCK_LIDAR_HPP_
