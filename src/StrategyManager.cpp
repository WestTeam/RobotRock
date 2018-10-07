// Copyright (c) 2018 All Rights Reserved WestBot

#include <memory>

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SystemManager.hpp>
//#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/WaitAction.hpp>
#include <WestBot/RobotRock/StrategyManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.StrategyManager" )
}

StrategyManager::StrategyManager(
    SystemManager& systemManager,
    TrajectoryManager& trajectoryManager,
    QObject* parent )
    : QObject( parent )
    , _systemManager( systemManager )
    , _trajectoryManager( trajectoryManager )
    , _currentAction( nullptr )
    , _stratIsRunning( false )
    , _color( Color::Unknown )
{
}

void StrategyManager::stop()
{
    tDebug( LOG ) << ">>> GAME ENDED";

    _actions.clear();
    _stratIsRunning = false;

    // Stop traj
    _trajectoryManager.hardStop();
    _trajectoryManager.disable();
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

    WaitAction::Ptr wait85s =
        std::make_shared< WaitAction >( 85 * 1000 );

    WaitAction::Ptr wait2s =
        std::make_shared< WaitAction >( 2 * 1000 );

  /*  MoveAction::Ptr moveTotem1 =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            600.0,
            500.0 * inv,
			false );
    */

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>><< OUR STRAT
    _actions.push_back( wait85s );
    _actions.push_back( wait2s );
    //_actions.push_back( moveTotem1 );
}

void StrategyManager::doStrat( const Color& color )
{
	// Build the strat for selected color
	buildStrat( color );

    tDebug( LOG ) << "Do strat for color:" << color;

    tDebug( LOG ) << ">>>>>>>> ACTIONS SIZE" << _actions.size();

	// Strat loop
	int i = 0;
	for( const auto& action: _actions )
	{
		_currentAction = action;
		action->execute();

		_actions.removeOne( action );
        tDebug( LOG ) << "Execute action number" << i;
		i++;

		/*if( ! _stratIsRunning )
		{
			qDebug() << QTime::currentTime().toString() << "Finish current action and stop after";
			break;
		}*/
	}

    tDebug( LOG )
		<< "Strat is over. Make sure we have clear the action list";

	_stratIsRunning = false;
	_actions.clear();
}
