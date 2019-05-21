// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ActionList.hpp>
#include <WestBot/RobotRock/StrategyManagerFoo.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.StrategyManagerFoo" )
}

StrategyManagerFoo::StrategyManagerFoo()
    : _trajectoryManager( nullptr )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _obstacleToClose( false )
    , _init( false )
{
}

bool StrategyManagerFoo::init( const TrajectoryManager::Ptr& trajectoryManager )
{
    if ( ! _init )
    {
        _trajectoryManager = trajectoryManager;
        _init = true;
    }
    else
    {
        tDebug( LOG ) << "StrategyManagerFoo already initialized";
    }

    return true;
}

void StrategyManagerFoo::deinit()
{
    _trajectoryManager = nullptr;
    _init = false;
}

void StrategyManagerFoo::stop()
{
    hardStop();

    // Stop traj
    _trajectoryManager->hardStop();
    _trajectoryManager->disable();
}

// Private methods
void StrategyManagerFoo::buildStrat( const Color& color )
{
    float inv = 1.0;
    float shift = 0.0;
    float offset = 0.0;

    if( color == Color::Yellow )
    {
        inv = -1.0;
        shift = 0.0;
        offset = 0.0;
    }
    else
    {
        inv = 1.0;
        shift = 0.0;
        offset = 0.0;
    }
/*
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( moveToCenterZone( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( turnA90( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( moveALittleForward( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( moveALittleForward2( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( moveToStartZone( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( orientationZoneDepose( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );

    _actions.push_back( moveALittleForward2( _trajectoryManager, inv ) );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
    _actions.push_back( wait500Ms() );
*/
    _stratIsRunning = true;
    _trajectoryManager->setAbort( false );
}

void StrategyManagerFoo::doStrat( const Color& color )
{
	// Build the strat for selected color
    buildStrat( color );

    tDebug( LOG ) << "Do strat for color:" << color;

    WaitAction::Ptr wait500ms =
        std::make_shared< WaitAction >( 500 );

    while( _stratIsRunning )
    {
        if( _obstacleToClose )
        {
            _trajectoryManager->hardStop();
            _trajectoryManager->setAbort(false);

            _actions.push_front( wait500ms );
        }

        if( _actions.size() != 0 )
        {
            _currentAction = _actions.takeFirst();
            _currentAction->execute();
        }
        else {
            tDebug( LOG ) << "NO MORE ACTIONS: Wait until the end";
            _actions.push_front( wait500ms );
        }
    }
}

void StrategyManagerFoo::hardStop()
{
    _trajectoryManager->setAbort( true );
    _stratIsRunning = false;
    _actions.clear();
}

void StrategyManagerFoo::obstacleToClose( bool avoid )
{
    _obstacleToClose = avoid;

    if( _obstacleToClose )
    {
        _trajectoryManager->setAbort( true );
    }
}
