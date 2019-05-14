// COPYRIGHT (C) 2019 ALL RIGHTS RESERVED WESTBot

#ifndef WESTBOT_ROBOTROCK_SYSTEMMANAGERSIMU_HPP_
#define WESTBOT_ROBOTROCK_SYSTEMMANAGERSIMU_HPP_

#include "GameThread.hpp"
#include "Odometry.hpp"
//#include "Lidar.hpp"
//#include "Monitoring.hpp"
//#include "Recalage.hpp"
//#include "SimTcpServer.hpp"
#include "StrategyManager.hpp"
#include "SystemManager.hpp"
#include "TrajectoryManager.hpp"
//#include "Vl6180x.hpp"

namespace WestBot {
namespace RobotRock {

class SystemManagerSimu : public SystemManager
{
public:
    SystemManagerSimu(
        const StrategyManager::Ptr& strategyManager,
        QObject* parent = nullptr );

    ~SystemManagerSimu() override;

    bool init() override;

    void start() override;
    void stop() override;
    void reset() override;

    bool isSafe() const override;

private:
    void initRecalage();

protected:
    void displayColor( const DigitalValue& value ) override;

private:
    QTimer _opponentTimer;

    // Base system
    Odometry::Ptr _odometry;
    //Recalage::Ptr _recalage;
    //LidarRPLidarA2::Ptr _lidar;
    TrajectoryManager::Ptr _trajectoryManager;
    StrategyManager::Ptr _strategyManager;
    //Monitoring::Ptr _monitoring;
    GameThread::Ptr _game;
    //Vl6180x _distanceSensor;
    //Experiment _experiment;
    //SimTcpServer _simServer;
};

}
}

#endif // WESTBOT_ROBOTROCK_SYSTEMMANAGERSIMU_HPP_
