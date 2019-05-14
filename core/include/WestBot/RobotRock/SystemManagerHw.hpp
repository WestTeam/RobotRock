// COPYRIGHT (C) 2019 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGERHW_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGERHW_HPP_

#include "Experiment.hpp"
#include "GameThread.hpp"
#include "Lidar.hpp"
#include "Monitoring.hpp"
#include "Recalage.hpp"
#include "SimTcpServer.hpp"
#include "StrategyManager.hpp"
#include "SystemManager.hpp"
#include "TrajectoryManager.hpp"
#include "Vl6180x.hpp"

namespace WestBot {
namespace RobotRock {

class SystemManagerHw : public SystemManager
{
public:
    enum class SystemMode
    {
        Free, // No PID -> register = 0x00
        DistanceOnly, // PIDD -> register = 0x01
        AngleOnly, // PIDA -> register = 0x02
        Full, // All PID -> register = 0x03
        Unused // 0xFF : Unused for now
    };

    SystemManagerHw(
        const Hal::Ptr& hal,
        const StrategyManager::Ptr& strategyManager );

    ~SystemManagerHw() override;

    bool init() override;

    void start() override;
    void stop() override;
    void reset() override;

    void setMode( SystemMode mode );
    SystemMode mode() const;

    bool isSafe() const override;

private:
    void initRecalage();

protected:
    void displayColor( const DigitalValue& value ) override;

private:
    Hal::Ptr _hal;
    QTimer _opponentTimer;

    SystemMode _systemMode;

    // Base system
    Odometry::Ptr _odometry;
    Recalage::Ptr _recalage;
    LidarRPLidarA2::Ptr _lidar;
    TrajectoryManager::Ptr _trajectoryManager;
    StrategyManager::Ptr _strategyManager;
    Monitoring::Ptr _monitoring;
    GameThread::Ptr _game;
    Vl6180x _distanceSensor;
    Experiment _experiment;
    SimTcpServer _simServer;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGERHW_HPP_
