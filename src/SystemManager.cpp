// Copyright (c) 2018 All Rights Reserved WestBot

#include <Macros.hpp>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/SystemManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.SystemManager" )
}

SystemManager::SystemManager( Hal& hal, QObject* parent )
    : QObject( parent )
    , _hal( hal )
    , _trajectoryManager( _hal, _recalage )
    , _systemMode( SystemManager::SystemMode::Full )
{
}

SystemManager::~SystemManager()
{
    stop();
}

//
// Public methods
//

bool SystemManager::init()
{
    // Always reset the system at startup
    reset();

    // Config PID Distance
    _hal._pidDistanceEnable.write( 0 );
    _hal._pidDistanceOverride.write( 0 );
    _hal._pidDistanceInverted.write( 0 );
    _hal._pidDistanceKp.write( ( float ) 2000.0 );
    _hal._pidDistanceKi.write( ( float ) 0.0 );
    _hal._pidDistanceKd.write( ( float ) 0.0 );

    _hal._pidDistanceSpeed.write( ( float ) 0.01 );
    _hal._pidDistanceAcceleration.write( ( float ) 0.0001 );
    _hal._pidDistanceSaturation.write( 25000 );

    _hal._pidDistanceTarget.write( _hal._pidDistancePosition.read< float >() );
    _hal._pidDistanceEnable.write( 1 );

    // Config PID Angle
    _hal._pidAngleEnable.write( 0 );
    _hal._pidAngleOverride.write( 0 );
    _hal._pidAngleInverted.write( 1 );
    _hal._pidAngleKp.write( ( float ) 500000.0 );
    _hal._pidAngleKi.write( ( float ) 0.0 );
    _hal._pidAngleKd.write( ( float ) 0.0 );

    _hal._pidAngleSpeed.write( ( float ) 0.0001 );
    _hal._pidAngleAcceleration.write( ( float ) 0.00000002 );
    _hal._pidAngleSaturation.write( 25000 );

    _hal._pidAngleTarget.write( _hal._pidAnglePosition.read< float >() );
    _hal._pidAngleEnable.write( 1 );

    if( ! _recalage.init( _hal ) )
    {
        tWarning( LOG ) << "Failed to init recalage module";
        return false;
    }

    _trajectoryManager.init();

    // Override output registers
    _hal._outputOverride.write( 0x01010101 );

    return true;
}

void SystemManager::start()
{
	tDebug( LOG ) << "System starting...";
}

void SystemManager::stop()
{
    tDebug( LOG ) << "System stopped";
}

void SystemManager::reset()
{
    _hal._resetAll.write( 1 );

    _hal.clearRegisters();

    _hal._resetAll.write( 0 );

    tDebug( LOG ) << "System was reset";
}

void SystemManager::setMode( SystemManager::SystemMode mode )
{
    _systemMode = mode;
}

SystemManager::SystemMode SystemManager::mode() const
{
    return _systemMode;
}
