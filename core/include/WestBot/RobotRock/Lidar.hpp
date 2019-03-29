// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_LIDAR_HPP_
#define WESTBOT_ROBOTROCK_LIDAR_HPP_

#include <QThread>

#include <WestBot/RobotRock/ItemRegister.hpp>
#include <WestBot/RobotRock/Odometry.hpp>

#include <WestBot/RPLidar/RPLidar.hpp>

class QString;

namespace WestBot {
namespace RobotRock {

typedef struct
{
    uint8_t quality; // 0 to 63
    double r; // mm
    double theta; // rad
    RobotPos pos;
} LidarData;

#define LIDAR_MAX_SCAN_POINTS 8192

class LidarBase
{
public:
    using Ptr = std::shared_ptr< LidarBase >;

    virtual void startMotor(float percentage) = 0;
    virtual void stopMotor()= 0;

    virtual void startScan()= 0;
    virtual void stopScan()= 0;

    virtual void setMinimumQuality(uint8_t minQuality) = 0;
    virtual bool get360ScanData(LidarData (&data)[LIDAR_MAX_SCAN_POINTS], uint32_t &count) = 0;
};

class LidarRPLidarA2 : public LidarBase
{
public:
    using Ptr = std::shared_ptr< LidarRPLidarA2 >;

    LidarRPLidarA2( const QString& lidarTTY = "/dev/ttyUSB0", const uint32_t baudrate = 256000, ItemRegister::Ptr Pwm = nullptr);
    ~LidarRPLidarA2();

    bool init();

    void startMotor(float percentage);
    void stopMotor();

    void startScan();
    void stopScan();

    void setMinimumQuality(uint8_t minQuality);

    bool get360ScanData(LidarData (&data)[LIDAR_MAX_SCAN_POINTS], uint32_t &count);

private:

    // cannot be called while scanning
    QString info();
    // cannot be called while scanning
    bool health();

    uint8_t _minQuality;
    RPLidar::RPLidar _lidar;
    ItemRegister::Ptr _pwm; // if null, use motor control from Lidar

    QMutex _lock;

};

}
}

#endif // WESTBOT_ROBOTROCK_LIDAR_HPP_
