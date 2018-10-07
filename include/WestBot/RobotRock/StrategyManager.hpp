// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

#include <QList>
#include <QString>

#include "Action.hpp"
#include "Common.hpp"
#include "TrajectoryManager.hpp"

namespace WestBot {
namespace RobotRock {

    class SystemManager;

/*!
* \brief This class manage the robot strategy by handling data from FPGA
*        and push action in the action queue of the game manager.
*/
class StrategyManager
{
public:
    StrategyManager(
        SystemManager& systemManager,
        TrajectoryManager& trajectoryManager );

    void stop();

    void doStrat( const Color& color );

    void buildStrat( const Color& color );

    void hardStop();

private:
    SystemManager& _systemManager;
    TrajectoryManager& _trajectoryManager;

    QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
    Color _color;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
