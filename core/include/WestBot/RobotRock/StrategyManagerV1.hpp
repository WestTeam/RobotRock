// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGERV1_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGERV1_HPP_

#include <QList>
#include <QTimer>

#include "Action.hpp"
#include "StrategyManager.hpp"
#include "AStarHighLevel.hpp"

namespace WestBot {
namespace RobotRock {

/*!
 * \brief StratV1 builder and launcher
 *
 */
class StrategyManagerV1 : public StrategyManager
{
public:
    StrategyManagerV1( QObject* parent = nullptr );

    ~StrategyManagerV1() override = default;

    bool init( const TrajectoryManager::Ptr& trajectoryManager ) override;

    void deinit() override;

    void stop() override;

    void doStrat( const Color& color ) override;

    void buildStrat( const Color& color ) override;

    void hardStop() override;

    void obstacleToClose( bool avoid ) override;

private:
    TrajectoryManager::Ptr _trajectoryManager; // Here we use a ref not a copy very important

    AStarHighLevel _astar;

    QList< Action::Ptr > _list1;
    QList< Action::Ptr > _list2;
    QList< Action::Ptr > _list3;
    QList< Action::Ptr > _avoidList;

    //QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
    bool _obstacleToClose;
    bool _init;

    QTimer _astarTimer;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGERV1_HPP_
