// Copyright (c) 2018-2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/TrajectoryManager.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.TrajectoryManager" )
}

TrajectoryManager::TrajectoryManager(
    const Hal::Ptr& hal,
    const Recalage::Ptr& recalage )
    : _hal( hal )
    , _recalage( recalage )
{
}

// We set some default values for speed and acceleration
void TrajectoryManager::init()
{
    _hal->_trajFreqHz.write( 10 );

    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_DISABLE );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 10 );
        tDebug( LOG ) << "wait cmd ack";
    }

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_WND );
    _hal->_trajCmdWndDistance.write( ( float ) 10.0 );
    _hal->_trajCmdWndAngleDeg.write( ( float ) 2.0 );
    _hal->_trajCmdWndAngleStartDeg.write( ( float ) 10.0 );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 10 );
        tDebug( LOG )
            << "wait cmd ack "
            << _hal->_trajOutAck.read< uint8_t >();
    }

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_DISTANCE );
    _hal->_trajCmdCfgSpeed.write( ( float ) 0.12 );
    _hal->_trajCmdCfgAcc.write( ( float ) 0.00004 );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 10 );
        tDebug( LOG ) << "wait cmd ack " << _hal->_trajOutAck.read< uint8_t >();
    }

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_ANGLE );
    _hal->_trajCmdCfgSpeed.write( ( float ) 0.0008 );
    _hal->_trajCmdCfgAcc.write( ( float) 0.0000002 );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 10 );
       tDebug( LOG ) << "wait cmd ack";
    }

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_ENABLE );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 10 );
       tDebug( LOG ) << "wait cmd ack";
    }

    tInfo( LOG ) << "Trajectory manager initialized";
}

void TrajectoryManager::waitTrajReady()
{
    do
    {
        QThread::msleep( 10 );
        /*
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();
        tDebug( LOG )
            << "Wait traj ready: State:" << state << "in windows:" << inWindow
            << "x/y/theta:" << _hal->_odometryX.read<int16_t>() << "/"
            << _hal->_odometryY.read<int16_t>() << "/"
            << _hal->_odometryTheta.read<int16_t>();
            */

    } while( !isTrajReady() );
}

bool TrajectoryManager::isTrajReady()
{
    uint8_t inWindow;
    TrajectoryManager::TrajectoryState state;

    state = static_cast< TrajectoryManager::TrajectoryState >(
        _hal->_trajOutState.read< uint8_t >() );
    inWindow = _hal->_trajOutInWindow.read< uint8_t >();
    tDebug( LOG )
        << "Wait traj ready: State:" << state << "in windows:" << inWindow
        << "x/y/theta:" << _hal->_odometryX.read<int16_t>() << "/"
        << _hal->_odometryY.read<int16_t>() << "/"
        << _hal->_odometryTheta.read<int16_t>();

    return (state == TrajectoryState::READY);
}

void TrajectoryManager::disable()
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_DISABLE );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 1 );
       tDebug( LOG ) << "wait disable ack";
    }
}

void TrajectoryManager::enable()
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_ENABLE );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 1 );
       tDebug( LOG ) << "wait enable ack";
    }
}

void TrajectoryManager::stop()
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_STOP );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 1 );
       tDebug( LOG ) << "wait stop ack";
    }
}

void TrajectoryManager::hardStop()
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_HARDSTOP );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 1 );
       tDebug( LOG ) << "wait hardstop ack";
    }
}

void TrajectoryManager::setDistanceConfig( float speed, float acc )
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_DISTANCE );
    _hal->_trajCmdCfgSpeed.write( speed );
    _hal->_trajCmdCfgAcc.write( acc );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 10 );
        tDebug( LOG )
            << "wait cmd ack " << _hal->_trajOutAck.read< uint8_t >();
    }
}

void TrajectoryManager::setAngleConfig( float speed, float acc )
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_ANGLE );
    _hal->_trajCmdCfgSpeed.write( speed );
    _hal->_trajCmdCfgAcc.write( acc );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
       QThread::msleep( 10 );
       tDebug( LOG ) << "wait cmd ack";
    }
}

void TrajectoryManager::setWindow(
    float distance,
    float angleDeg,
    float startAngleDeg )
{
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_CFG_WND );
    _hal->_trajCmdWndDistance.write( distance );
    _hal->_trajCmdWndAngleDeg.write( angleDeg );
    _hal->_trajCmdWndAngleStartDeg.write( startAngleDeg );
    _hal->_trajCmdValid.write( 0x1 );

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 10 );
        tDebug( LOG )
            << "wait cmd ack "
            << _hal->_trajOutAck.read< uint8_t >();
    }
}

// Trajectories: all this method are blocking
void TrajectoryManager::moveDRel(
    float distance,
    bool correction,
    bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_D_REL );
    _hal->_trajCmdADDistance.write( distance );

    correction
        ?  _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();
        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y:" << _hal->_odometryX.read<int16_t>() << "/"
                << _hal->_odometryY.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveOnlyDRel(
    float distance,
    bool correction,
    bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_ONLY_D_REL );
    _hal->_trajCmdADDistance.write( distance );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y:" << _hal->_odometryX.read<int16_t>() << "/"
                << _hal->_odometryY.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnARel(
    float theta,
    bool correction,
    bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_A_REL );
    _hal->_trajCmdADAngleDeg.write( theta );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    int i = 0;
    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

    do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "theta:" << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnAAbs(
    float theta,
    bool correction,
    bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = theta;
    currentPos.x = 0;
    currentPos.y = 0;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_A_ABS );
    _hal->_trajCmdADAngleDeg.write( ( float ) pos.theta );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "theta:" << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnOnlyARel(
    float theta,
    bool correction,
    bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_ONLY_A_REL );
    _hal->_trajCmdADAngleDeg.write( theta );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "theta:" << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnOnlyAAbs(
    float theta,
    bool correction,
    bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = theta;
    currentPos.x = 0;
    currentPos.y = 0;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_ONLY_A_ABS );
    _hal->_trajCmdADAngleDeg.write( ( float ) pos.theta );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "theta:" << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnToXY( float x, float y, bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = 0;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_TURNTO_XY );
    _hal->_trajCmdPosX.write( ( float ) pos.x );
    _hal->_trajCmdPosY.write( ( float ) pos.y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y:" << _hal->_odometryX.read<int16_t>() << "/"
                << _hal->_odometryY.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::turnToXYBehind( float x, float y, bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = 0;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_TURNTO_XY_BEHIND );
    _hal->_trajCmdPosX.write( ( float ) pos.x );
    _hal->_trajCmdPosY.write( ( float ) pos.y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y:" << _hal->_odometryX.read<int16_t>() << "/"
                << _hal->_odometryY.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = theta;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_GOTO_XY_ABS );
    _hal->_trajCmdPosTheta.write( ( float ) pos.theta );
    _hal->_trajCmdPosX.write( ( float ) pos.x );
    _hal->_trajCmdPosY.write( ( float ) pos.y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y/theta:" << _hal->_odometryX.read<int16_t>()
                << "/" << _hal->_odometryY.read<int16_t>() << "/"
                << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveForwardToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = theta;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_GOTO_FORWARD_XY_ABS );
    _hal->_trajCmdPosTheta.write( ( float ) pos.theta );
    _hal->_trajCmdPosX.write( ( float ) pos.x );
    _hal->_trajCmdPosY.write( ( float ) pos.y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y/theta:" << _hal->_odometryX.read<int16_t>()
                << "/" << _hal->_odometryY.read<int16_t>() << "/"
                << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveBackwardToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = theta;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_GOTO_BACKWARD_XY_ABS );
    _hal->_trajCmdPosTheta.write( ( float ) pos.theta );
    _hal->_trajCmdPosX.write( ( float ) pos.x );
    _hal->_trajCmdPosY.write( ( float ) pos.y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y/theta:" << _hal->_odometryX.read<int16_t>()
                << "/" << _hal->_odometryY.read<int16_t>() << "/"
                << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveToDARel(
    float theta,
    float distance,
    bool correction,
    bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_D_A_REL );
    _hal->_trajCmdADAngleDeg.write( theta );
    _hal->_trajCmdADDistance.write( distance );

    correction
        ? _hal->_trajCmdADCorrection.write( 1 )
        : _hal->_trajCmdADCorrection.write( 0 );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y/theta:" << _hal->_odometryX.read<int16_t>()
                << "/" << _hal->_odometryY.read<int16_t>() << "/"
                << _hal->_odometryTheta.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}

void TrajectoryManager::moveToXYRel( float x, float y, bool doNotBlock )
{
    uint8_t inWindow;
    uint8_t commandId = _hal->_trajOutAck.read< uint8_t >();

    _hal->_trajCmdValid.write( 0x0 );
    _hal->_trajCmdId.write( commandId++ );
    _hal->_trajCmdType.write( CMD_TYPE_TRAJ );
    _hal->_trajCmdOrderType.write( TRAJ_GOTO_XY_REL );
    _hal->_trajCmdPosX.write( x );
    _hal->_trajCmdPosY.write( y );

    _hal->_trajCmdValid.write( 0x1 ) ;

    TrajectoryManager::TrajectoryState state;

    while( _hal->_trajOutAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }

	if( doNotBlock )
    {
        return;
    }

	int i = 0;
	do
    {
        QThread::msleep( 10 );
        state = static_cast< TrajectoryManager::TrajectoryState >(
            _hal->_trajOutState.read< uint8_t >() );
        inWindow = _hal->_trajOutInWindow.read< uint8_t >();

        if( ( i++ % 100 ) == 0 )
        {
            tDebug( LOG )
                << "Wait traj ready: State:" << state << "in windows:"
                << inWindow << "x/y:" << _hal->_odometryX.read<int16_t>() << "/"
                << _hal->_odometryY.read<int16_t>();
        }
	} while( state != TrajectoryState::READY );
}
