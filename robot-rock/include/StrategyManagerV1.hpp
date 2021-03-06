// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGERV1_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGERV1_HPP_

#include <QList>
#include <QTimer>

#include <WestBot/RobotRock/Action.hpp>
#include <WestBot/RobotRock/AStarHighLevel.hpp>
#include <WestBot/RobotRock/StrategyManager.hpp>

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

    bool init(
            const Odometry::Ptr& odometry,
            const Recalage::Ptr& recalage,
            const ArmsManager::Ptr& armsManager,
            const OpponentDetection::Ptr opponentDetection,
            const TrajectoryManager::Ptr& trajectoryManager ) override;

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
    Recalage::Ptr _recalage;
    ArmsManager::Ptr _armsManager;
    OpponentDetection::Ptr _opponentDetection;
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
