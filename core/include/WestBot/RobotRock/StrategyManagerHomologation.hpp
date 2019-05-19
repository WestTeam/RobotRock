// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGERHOMOLAGATION_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGERHOMOLAGATION_HPP_

#include <QList>

#include "Action.hpp"
#include "StrategyManager.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief This class contains the strat for our homologation.
 */
class StrategyManagerHomologation : public StrategyManager
{
public:
    StrategyManagerHomologation();

    ~StrategyManagerHomologation() override = default;

    bool init( const TrajectoryManager::Ptr& trajectoryManager ) override;

    void deinit() override;

    void stop() override;

    void doStrat( const Color& color ) override;

    void buildStrat( const Color& color ) override;

    void hardStop() override;

    void obstacleToClose( bool avoid ) override;

private:
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
