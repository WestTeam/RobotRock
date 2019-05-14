// Copyright (c) 2019 All Rights Reserved WestBot

#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/Odometry.hpp>
#include <WestBot/RobotRock/TrajectoryManagerSimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY(
        LOG,
        "WestBot.RobotRock.TrajectoryManagerSimu" )
}

TrajectoryManagerSimu::TrajectoryManagerSimu( const Odometry::Ptr& odometry )
    : _odometry( odometry )
    , _abort (false)
{
    _trajState = TrajectoryState::READY;
    _commandId = 0;
}

// We set some default values for speed and acceleration
void TrajectoryManagerSimu::init()
{
    _enabled = false;

    _wndDist = 10.0;
    _wndAngleDeg = 2.0;
    _wndStartAngleDeg = 10.0;

    _distSpeed = 0.12;
    _distAcc = 0.00004;
    _angleSpeed = 0.0008;
    _angleAcc = 0.0000002;


    _targetPos = _odometry->getPosition();

    // start thread
    start();

    tInfo( LOG ) << "Trajectory manager initialized";


}


void TrajectoryManagerSimu::waitTrajReady()
{
    do
    {
        QThread::msleep( 10 );
    } while( !isTrajReady() && !_abort );

    _abort = false;
}

bool TrajectoryManagerSimu::isTrajReady()
{
    TrajectoryManager::TrajectoryState state = _trajState;

    tDebug( LOG )
        << "Wait traj ready: State:" << state
        << "x/y/theta:" << _odometry->getPosition().x << "/"
        << _odometry->getPosition().y << "/" << "/"
        << DEG(_odometry->getPosition().theta);

    return (state == TrajectoryState::READY);
}

void TrajectoryManagerSimu::disable()
{
    _enabled = false;
}

void TrajectoryManagerSimu::enable()
{
    _enabled = true;

}

void TrajectoryManagerSimu::stop()
{

    _trajType = TrajectoryType::TYPE_TRAJ_STOP;

}

void TrajectoryManagerSimu::hardStop()

{
    _trajType = TrajectoryType::TYPE_TRAJ_HARDSTOP;

}

void TrajectoryManagerSimu::setAbort( bool abort )
{
    _abort = abort;
}

void TrajectoryManagerSimu::setDistanceConfig( float speed, float acc )
{
    _distSpeed = speed;
    _distAcc = acc;
}

void TrajectoryManagerSimu::setAngleConfig( float speed, float acc )
{
    _angleSpeed = speed;
    _angleAcc = acc;
}

void TrajectoryManagerSimu::setWindow(
    float distance,
    float angleDeg,
    float startAngleDeg )
{
    _wndDist = distance;
    _wndAngleDeg = angleDeg;
    _wndStartAngleDeg = startAngleDeg;

}

void TrajectoryManagerSimu::moveDRel(
    float distance,
    bool correction,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_D_REL;
    _commandDistance = distance;
    _commandId++;

    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveOnlyDRel(
    float distance,
    bool correction,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_ONLY_D_REL;
    _commandDistance = distance;
    _commandId++;

    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnARel(
    float theta,
    bool correction,
    bool doNotBlock )
{
    _trajType = TrajectoryType::TYPE_TRAJ_A_REL;
    _commandAngle = theta;
    _commandId++;

    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnAAbs(
    float theta,
    bool correction,
    bool doNotBlock )
{
    _trajType = TrajectoryType::TYPE_TRAJ_A_ABS;
    _commandAngle = theta;
    _commandId++;


    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnOnlyARel(
    float theta,
    bool correction,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_ONLY_A_REL;
    _commandAngle = theta;
    _commandId++;

    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnOnlyAAbs(
    float theta,
    bool correction,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_ONLY_A_ABS;
    _commandAngle = theta;
    _commandId++;


    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnToXY( float x, float y, bool doNotBlock )
{
    RobotPos currentPos;
    currentPos.theta = 0;
    currentPos.x = x;
    currentPos.y = y;

    RobotPos pos = currentPos; // RobotPos pos = _recalage->sendPos( currentPos );

    _trajType = TrajectoryType::TYPE_TRAJ_TURNTO_XY;
    _commandX = x;
    _commandY = y;
    _commandId++;


    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::turnToXYBehind( float x, float y, bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_TURNTO_XY_BEHIND;
    _commandX = x;
    _commandY = y;
    _commandId++;

    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{
    _trajType = TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS;
    _commandAngle = theta; // USED??
    _commandX = x;
    _commandY = y;
    _commandId++;

    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveForwardToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{
    _trajType = TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS;
    _commandAngle = theta; // USED??
    _commandX = x;
    _commandY = y;
    _commandId++;

    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveBackwardToXYAbs(
    float theta,
    float x,
    float y,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS;
    _commandAngle = theta; // USED??
    _commandX = x;
    _commandY = y;
    _commandId++;

    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveToDARel(
    float theta,
    float distance,
    bool correction,
    bool doNotBlock )
{

    _trajType = TrajectoryType::TYPE_TRAJ_D_A_REL;
    _commandAngle = theta;
    _commandDistance = distance;
    _commandId++;

    TrajectoryManager::TrajectoryState state;


	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::moveToXYRel( float x, float y, bool doNotBlock )
{
    _trajType = TrajectoryType::TYPE_TRAJ_GOTO_XY_REL;
    _commandX = x;
    _commandY = y;
    _commandId++;



    TrajectoryManager::TrajectoryState state;

	if( doNotBlock )
    {
        return;
    }

    int i = 0;
    do
    {
        QThread::msleep( 10 );
        state = _trajState;
        if( ( i++ % 100 ) == 0 )
        {
            isTrajReady();
        }
    } while( state != TrajectoryState::READY && !_abort);

    _abort = false;
}

void TrajectoryManagerSimu::run()
{
    uint8_t localCommandId = 0;

    do {
        QThread::msleep(10);

        switch (_trajState)
        {
            case TrajectoryManager::TrajectoryState::READY:
                if (localCommandId != _commandId)
                {
                    switch (_trajType)
                    {
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_STOP:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_HARDSTOP:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS:
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS:

                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL:
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL:

                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL:
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_REL:

                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY_BEHIND:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_REL:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS:
                            break;
                        case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS:
                            break;
                        break;
                    }


                    if (_trajType == TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS)
                    {
                        RobotPos newPos;

                        newPos.x = _commandX;
                        newPos.y = _commandY;

                        _odometry->setPosition(newPos);

                        tDebug( LOG ) << "Traj Command Executed:" << _trajType << _commandX << _commandY;
                    }

                    localCommandId = _commandId;
                }
                break;

            case TrajectoryManager::TrajectoryState::RUNNING_XY_START:
                break;
            case TrajectoryManager::TrajectoryState::RUNNING_XY_ANGLE:
                break;

            case TrajectoryManager::TrajectoryState::RUNNING_XY_ANGLE_OK:
                break;
        }
    } while (true);
}
