// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ActionList.hpp>
#include <WestBot/RobotRock/StrategyManagerV1.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.StrategyManagerV1" )
}

StrategyManagerV1::StrategyManagerV1()
    : _trajectoryManager( nullptr )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _obstacleToClose( false )
    , _init( false )
{
}

bool StrategyManagerV1::init( const TrajectoryManager::Ptr& trajectoryManager )
{
    if ( ! _init )
    {
        _trajectoryManager = trajectoryManager;
        _init = true;
    }
    else
    {
        tDebug( LOG ) << "StrategyManagerV1 already initialized";
    }

    return true;
}

void StrategyManagerV1::deinit()
{
    _trajectoryManager = nullptr;
    _init = false;
}

void StrategyManagerV1::stop()
{
    hardStop();

    // Stop traj
    _trajectoryManager->hardStop();
    _trajectoryManager->disable();
}

// Private methods
void StrategyManagerV1::buildStrat( const Color& color )
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

    // List 1: On va chercher le goldenium
    // On prend les 2 pucks rouge et on fonce a l'accelerateur
    _list1.push_back( moveToAccelerator( _trajectoryManager, inv ) );
    _list1.push_back( moveToAccelerator2( _trajectoryManager, inv ) );
    _list1.push_back( moveToAccelerator3( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance2( _trajectoryManager, inv ) );
    _list1.push_back( moveToCenterZone2( _trajectoryManager, inv ) );
    _list1.push_back( onSaqueDedans( _trajectoryManager, inv ) );
    _list1.push_back( onSaqueDedans2( _trajectoryManager, inv ) );
    _list1.push_back( moveToAccelerator( _trajectoryManager, inv ) );
    _list1.push_back( moveToAccelerator2( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance2( _trajectoryManager, inv ) );
    _list1.push_back( goingBackALittle( _trajectoryManager, inv ) );
    _list1.push_back( getHighPucks( _trajectoryManager, inv ) );
    _list1.push_back( getHighPucks2( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance( _trajectoryManager, inv ) );
    _list1.push_back( moveToBalance2( _trajectoryManager, inv ) );
    _list1.push_back( moveToCenterZone2( _trajectoryManager, inv ) );
    _list1.push_back( moveToStartZone( _trajectoryManager, inv ) );
    _list1.push_back( orientationZoneDepose( _trajectoryManager, inv ) );
    _list1.push_back( moveALittleForward3( _trajectoryManager, inv ) );

    _stratIsRunning = true;
    _trajectoryManager->setAbort( false );
}

void StrategyManagerV1::doStrat( const Color& color )
{
	// Build the strat for selected color
    buildStrat( color );

    tDebug( LOG ) << "Do strat for color:" << color;

    while( _stratIsRunning )
    {
        if( _obstacleToClose )
        {
            _trajectoryManager->hardStop();
            _trajectoryManager->setAbort( false );

            _avoidList.push_front( wait500Ms() );
        }

        if( _avoidList.size() != 0 )
        {
            _currentAction = _avoidList.takeFirst();
            _currentAction->execute();
        }
        else if( _list1.size() != 0 )
        {
            _currentAction = _list1.takeFirst();
            _currentAction->execute();
        }
        else if( _list2.size() != 0 )
        {
            _currentAction = _list2.takeFirst();
            _currentAction->execute();
        }
        else if( _list3.size() != 0 )
        {
            _currentAction = _list3.takeFirst();
            _currentAction->execute();
        }
        else
        {
            tDebug( LOG ) << "NO MORE ACTIONS: Wait until the end";
            _list1.push_front( wait500Ms() );
        }
    }
}

void StrategyManagerV1::hardStop()
{
    _trajectoryManager->setAbort( true );
    _stratIsRunning = false;
    _list1.clear();
    _list2.clear();
    _list3.clear();
    _avoidList.clear();
}

void StrategyManagerV1::obstacleToClose( bool avoid )
{
    _obstacleToClose = avoid;

    if( _obstacleToClose )
    {
        _trajectoryManager->setAbort( true );
    }
}
