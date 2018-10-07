// Copyright (c) 2018 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGER_HPP_

#include <QList>
#include <QObject>
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
class StrategyManager : public QObject
{
public:
    StrategyManager(
        SystemManager& systemManager,
        TrajectoryManager& trajectoryManager,
        QObject* parent = nullptr );
    ~StrategyManager() override = default;

    void stop();

    void doStrat( const Color& color );

    void buildStrat( const Color& color );

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
