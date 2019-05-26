// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGERHOMOLAGATION_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGERHOMOLAGATION_HPP_

#include <QList>

#include <WestBot/RobotRock/Action.hpp>
#include <WestBot/RobotRock/StrategyManager.hpp>

namespace WestBot {
namespace RobotRock {

/*!
 * \brief This class contains the strat for our homologation.
 */
class StrategyManagerHomologation : public StrategyManager
{
public:
    StrategyManagerHomologation( QObject* parent = nullptr );

    ~StrategyManagerHomologation() override = default;

    bool init(
        const Odometry::Ptr& odometry,
        const TrajectoryManager::Ptr& trajectoryManager ) override;

    void deinit() override;

    void stop() override;

    void doStrat( const Color& color ) override;

    void buildStrat( const Color& color ) override;

    void hardStop() override;

    void obstacleAt( double xStart, double yStart, double xEnd, double yEnd ) override;

private:
    Odometry::Ptr _odometry;
    TrajectoryManager::Ptr _trajectoryManager; // Here we use a ref not a copy very important

    QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
    bool _obstacleToClose;
    bool _init;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGERHOMOLAGATION_HPP_
