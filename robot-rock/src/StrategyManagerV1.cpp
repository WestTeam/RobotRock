// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/ActionList.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>

#include "StrategyManagerV1.hpp"

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.StrategyManagerV1" )

    // TEST FOR A*
    MoveAction::Ptr moveGenericAction(
        const TrajectoryManager::Ptr& trajectoryManager,
        double x,
        double y,
        float inv )
    {
        return std::make_shared< MoveAction >(
            trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS,
            0.0,
            0.0,
            x,
            y,
            true );
    }
}

StrategyManagerV1::StrategyManagerV1( QObject* parent )
    : _trajectoryManager( nullptr )
    , _astar( _trajectoryManager, 1.0, 67, 100 )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _obstacleToClose( false )
    , _init( false )
{
    _astar.setCurrentPos( 19, 10 );
    _astar.setTarget( 50, 80 );
    //_astar.setObstacle( 20, 20, 60, 60 );

    _astar.setObstacle( 23, 23, 40, 40 );
    _astar.setObstacle( 23, 45, 50, 50 );
    _astar.setObstacle( 51, 30, 55, 50 );

    _astarTimer.setInterval( 10000 );

    connect(
        & _astar,
        & AStarHighLevel::newRoute,
        this,
        [ this ]( QList< QPair< uint, uint > > actions )
        {
            tDebug( LOG ) << "NEW ACTIONS LIST TO AVOID OBSTACLE" << actions.size();
            for( auto pathIt = actions.begin(); pathIt != actions.end(); ++pathIt )
            {
                tDebug( LOG ) << "Actions pos:" << pathIt->first << pathIt->second;
                _avoidList.push_back(
                    moveGenericAction( _trajectoryManager,
                   ( ( pathIt->first - 2.51 ) * 30 ),
                   -1.0 * ( 1500 - ( ( pathIt->second + 2.51 ) * 30 ) ),
                   1.0 ) );
            }
        } );

    connect(
        & _astarTimer,
        & QTimer::timeout,
        this,
        [ this ]()
        {
            _astar.processCurrentRoute( false );
            _astar.dumpMap();
        } );

    _astarTimer.setSingleShot( true );
    _astarTimer.start();
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

    _list1.push_back( wait500Ms() );
    _list1.push_back( wait500Ms() );
    _list1.push_back( wait500Ms() );
    _list1.push_back( wait500Ms() );

    // List 1: On va chercher le goldenium
    // On prend les 2 pucks rouge et on fonce a l'accelerateur
    /*_list1.push_back( moveToAccelerator( _trajectoryManager, inv ) );
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
    */
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
            tDebug( LOG ) << "WE HAVE A LIST TO AVOID THE OBSTACLE";
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
