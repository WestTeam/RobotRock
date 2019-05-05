// Copyright (c) 2018 All Rights Reserved WestBot

#include <memory>

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/StrategyManager.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>
#include <WestBot/RobotRock/WaitAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.StrategyManager" )
}

StrategyManager::StrategyManager( const TrajectoryManager::Ptr& trajectoryManager )
    : _trajectoryManager( trajectoryManager )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _obstacleToClose( false )
{
}

void StrategyManager::stop()
{
    hardStop();

    // Stop traj
    _trajectoryManager->hardStop();
    _trajectoryManager->disable();
}

// Private methods
void StrategyManager::buildStrat( const Color& color )
{
    float inv = 1.0;
    float shift = 0.0;
    float offset = 0.0;

    if( color == Color::Yellow )
    {
        inv = 1.0;
        shift = 20.0;
    }
    else
    {
        inv = -1.0;
        shift = -18.0;
        offset = 90.0;
    }

    MoveAction::Ptr move1 =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
            0.0,
            100000.0,
            0.0,
            0.0 * inv,
            true );

    // Our strat begins here
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );
    _actions.push_back( move1 );

    _stratIsRunning = true;
    _trajectoryManager->setAbort( false );
}

void StrategyManager::doStrat( const Color& color )
{
	// Build the strat for selected color
	buildStrat( color );

    tDebug( LOG ) << "Do strat for color:" << color;

    WaitAction::Ptr wait500ms =
        std::make_shared< WaitAction >( 100 );

    while( _stratIsRunning )
    {
        if( _obstacleToClose )
        {
            _trajectoryManager->hardStop();
            _trajectoryManager->setAbort(false);

            _actions.push_front( wait500ms );
        }

        _currentAction = _actions.takeFirst();
        _currentAction->execute();
    }
}

void StrategyManager::hardStop()
{
    _trajectoryManager->setAbort( true );
    _stratIsRunning = false;
    _actions.clear();
}

void StrategyManager::obstacleToClose( bool avoid )
{
    _obstacleToClose = avoid;

    if( _obstacleToClose )
    {
        _trajectoryManager->setAbort( true );
    }
}
