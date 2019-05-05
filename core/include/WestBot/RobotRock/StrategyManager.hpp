// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

#include <memory>

#include <QList>
#include <QString>

#include "Action.hpp"
#include "Common.hpp"
#include "TrajectoryManager.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief This class manage the robot strategy by handling data from FPGA
 *        and push action in the action queue.
 */
class StrategyManager
{
public:
    using Ptr = std::shared_ptr< StrategyManager >;

    StrategyManager( const TrajectoryManager::Ptr& trajectoryManager );

    void stop();

    void doStrat( const Color& color );

    void buildStrat( const Color& color );

    void hardStop();

    void obstacleToClose( bool avoid );

private:
    TrajectoryManager::Ptr _trajectoryManager; // Here we use a ref not a copy very important

    QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
    bool _obstacleToClose;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
