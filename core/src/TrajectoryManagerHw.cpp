// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Odometry.hpp>
#include <WestBot/RobotRock/TrajectoryManagerHw.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.TrajectoryManagerHw" )
}

TrajectoryManagerHw::TrajectoryManagerHw( const Hal::Ptr& hal )
    : _hal( hal )
    , _abort (false)
{
}

// We set some default values for speed and acceleration
void TrajectoryManagerHw::init()
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

void TrajectoryManagerHw::waitTrajReady()
{
    do
    {
        QThread::msleep( 10 );
    } while( !isTrajReady() && !_abort );

    _abort = false;
}

bool TrajectoryManagerHw::isTrajReady()
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

void TrajectoryManagerHw::disable()
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

void TrajectoryManagerHw::enable()
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

void TrajectoryManagerHw::stop()
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

void TrajectoryManagerHw::hardStop()
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

void TrajectoryManagerHw::setAbort( bool abort )
{
    _abort = abort;
}

void TrajectoryManagerHw::setDistanceConfig( float speed, float acc )
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

void TrajectoryManagerHw::setAngleConfig( float speed, float acc )
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

void TrajectoryManagerHw::setWindow(
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
void TrajectoryManagerHw::moveDRel(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveOnlyDRel(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnARel(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnAAbs(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnOnlyARel(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnOnlyAAbs(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnToXY( float x, float y, bool doNotBlock )
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::turnToXYBehind( float x, float y, bool doNotBlock )
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveToXYAbs(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveForwardToXYAbs(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveBackwardToXYAbs(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveToDARel(
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerHw::moveToXYRel( float x, float y, bool doNotBlock )
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
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}
