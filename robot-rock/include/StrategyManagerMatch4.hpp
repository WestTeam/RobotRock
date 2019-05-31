// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_STRATEGYMANAGERMATCH4_HPP_
#define WESTBOT_ROBOTROCK_STRATEGYMANAGERMATCH4_HPP_

#include <QList>

#include <WestBot/RobotRock/Action.hpp>
#include <WestBot/RobotRock/StrategyManager.hpp>

namespace WestBot {
namespace RobotRock {

/*!
 * \brief This class contains the strat for our homologation.
 */
class StrategyManagerMatch4 : public StrategyManager
{
public:
    StrategyManagerMatch4( QObject* parent = nullptr );

    ~StrategyManagerMatch4() override = default;

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

    QList< Action::Ptr > _actions;
    Action::Ptr _currentAction;
    bool _stratIsRunning;
    bool _obstacleToClose;
    bool _init;
    bool _invArms;

    QList< PuckPos* > _puckList;
};

}
}

#endif // WESTBOT_ROBOTROCK_STRATEGYMANAGERMATCH4_HPP_
