// Copyright (c) 2018-2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

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
    StrategyManager( TrajectoryManager& trajectoryManager );

    void stop();

    void doStrat( const Color& color );

    void buildStrat( const Color& color );

    void hardStop();

private:
    TrajectoryManager& _trajectoryManager; // Here we use a ref not a copy very important

    QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
