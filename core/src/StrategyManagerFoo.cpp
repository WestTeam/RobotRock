// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/StrategyManagerFoo.hpp>
#include <WestBot/RobotRock/SystemManager.hpp>
#include <WestBot/RobotRock/WaitAction.hpp>

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
    tDebug( LOG ) << "HERE 3";

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
        inv = 1.0;
        shift = 20.0;
    }
    else
    {
        inv = 1.0;
        shift = -18.0;
        offset = 90.0;
    }

    WaitAction::Ptr wait500Ms =
        std::make_shared< WaitAction >( 500 );

    WaitAction::Ptr wait200Ms =
        std::make_shared< WaitAction >( 200 );

    WaitAction::Ptr wait100Ms =
        std::make_shared< WaitAction >( 100 );

    // >>>>>>>>>>>>> MOVE ACTIONS
    MoveAction::Ptr waitTrajReady =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::WAIT_TRAJ_READY,
            0.0,
            0.0,
            0.0,
            0.0,
            false );

    MoveAction::Ptr moveTotem1 =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            600.0,
            500.0 * inv,
            false );

    MoveAction::Ptr moveTotem2 =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            ( 1100.0 - 162.0 ),
            ( 1000.0 - 162.0 ) * inv,
            false,
            false );

    MoveAction::Ptr moveAlignementDepose =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            ( 1100.0 + shift ),
            ( 900.0 + shift ) * inv,
            false );

    MoveAction::Ptr moveDepose =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            (1400.0+shift) - 170.0,
            ( (600.0+shift) + 170.0 ) * inv,
            false );

    MoveAction::Ptr moveDeposePlus =
         std::make_shared< MoveAction >(
                _trajectoryManager,
                TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
                0.0,
                35.0,
                0.0,
                0.0,
                false );

    MoveAction::Ptr avance95SansCorrection =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            175.0,
            0.0,
            0.0,
            false );

    MoveAction::Ptr avance95SansCorrectionPlus =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            175.0 + 40.0,
            0.0,
            0.0,
            false );

    MoveAction::Ptr recul180AvecCorrection =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            -180.0,
            0.0,
            0.0,
            true );

    MoveAction::Ptr recul180AvecCorrectionPlus =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            -180.0 - 40.0,
            0.0,
            0.0,
            true );

    MoveAction::Ptr recul50AvecCorrection =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            -50.0,
            0.0,
            0.0,
            true );

    MoveAction::Ptr turnA45 =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS,
            -45.0 * inv,
            0.0,
            0.0,
            0.0,
            true );

    MoveAction::Ptr moveTotemUpper =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            1850.0 - 150.0,
            ( 700.0 + 50.0 ) * inv,
            true );

    MoveAction::Ptr moveBackBeforeEject =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
            0.0,
            0.0,
            ( 1100.0 + shift ),
            ( 900.0 + shift ) * inv,
            false );

    MoveAction::Ptr turnToTotemBottom =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY,
            0.0,
            0.0,
            ( 790.0 ),
            ( 1500.0 - 365.0 ) * inv,
            false );

    MoveAction::Ptr moveToTotemBottom =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
            0.0,
            0.0,
            ( 790.0 ),
            ( 1500.0 - 365.0 ) * inv,
            false );

    MoveAction::Ptr safeBackTotemBottom =
        std::make_shared< MoveAction >(
            _trajectoryManager,
            TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL,
            0.0,
            -400.0,
            0.0,
            0.0,
            false );

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>><< OUR STRAT
    //_actions.push_back( mediumSpeedDistance );
    //_actions.push_back( windowCourbe );
    _actions.push_back( wait200Ms );
    _actions.push_back( moveTotem1 );
    _actions.push_back( wait200Ms );
    _actions.push_back( wait200Ms );
    _actions.push_back( wait200Ms );
    _actions.push_back( moveTotem2 );
    _actions.push_back( wait200Ms );
    //actions.push_back( wait200Ms );
    _actions.push_back( wait200Ms );
    //_actions.push_back( normalSpeedDistance );
    //_actions.push_back( windowPrecise );
    _actions.push_back( moveAlignementDepose );
    _actions.push_back( moveDepose );
    _actions.push_back( turnA45 );
    _actions.push_back( wait500Ms );

    _actions.push_back( wait500Ms );
    _actions.push_back( wait500Ms );
// we redo the action to be safe
    _actions.push_back( wait500Ms );
    _actions.push_back( wait500Ms );

    _actions.push_back( wait200Ms );

    _actions.push_back( recul180AvecCorrection ); // Deplacement -180
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( avance95SansCorrection );
    _actions.push_back( recul180AvecCorrection ); // Deplacement -80
    _actions.push_back( wait500Ms );
    _actions.push_back( moveDepose ); // Recallage sur position connue

    // On fait le deuxieme totem
    _actions.push_back( turnA45 );
    _actions.push_back( wait500Ms );

    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( wait200Ms );

    _actions.push_back( recul180AvecCorrection ); // Deplacement -180
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( avance95SansCorrection );
    _actions.push_back( recul180AvecCorrection ); // Deplacement -80
    _actions.push_back( wait200Ms );
    _actions.push_back( moveDepose ); // Recallage sur position connue

    // On fait le troisieme totem
    _actions.push_back( turnA45 );
    _actions.push_back( wait500Ms );

    _actions.push_back( wait200Ms );
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( wait200Ms );

    _actions.push_back( recul180AvecCorrection ); // Deplacement -180
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( avance95SansCorrection );
    _actions.push_back( recul180AvecCorrection ); // Deplacement -80
    _actions.push_back( wait200Ms );

    // Totem upper
    //_actions.push_back( fastSpeedDistance );
    _actions.push_back( moveTotemUpper );
    //_actions.push_back( windowCourbe );
    _actions.push_back( moveBackBeforeEject);
    //_actions.push_back( normalSpeedDistance );
    //_actions.push_back( windowPrecise );
    _actions.push_back( moveDepose ); // Recallage sur position connue
    _actions.push_back( turnA45 );
    _actions.push_back( wait500Ms );
    // we redo the action to be safe

    _actions.push_back( wait200Ms );
    _actions.push_back( recul180AvecCorrectionPlus ); // Deplacement -180
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( avance95SansCorrectionPlus );
    _actions.push_back( recul180AvecCorrectionPlus ); // Deplacement -80

    // Totem bottom
    //_actions.push_back( fastSpeedDistance );
    _actions.push_back( turnToTotemBottom );
    _actions.push_back( moveToTotemBottom );
    //_actions.push_back( windowCourbe );
    _actions.push_back( safeBackTotemBottom );
    //_actions.push_back( mediumSpeedDistance );
    //_actions.push_back( windowPrecise );
    _actions.push_back( moveDepose );
    //_actions.push_back( moveDeposePlus );
    _actions.push_back( turnA45 );
    _actions.push_back( wait500Ms );
    // we redo the action to be safe
    _actions.push_back( wait500Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( wait200Ms );
    _actions.push_back( recul180AvecCorrectionPlus ); // Deplacement -180
    _actions.push_back( wait200Ms );
    _actions.push_back( wait500Ms );
    _actions.push_back( avance95SansCorrectionPlus );
    _actions.push_back( recul180AvecCorrectionPlus ); // Deplacement -80

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
