// Copyright (c) 2018 All Rights Reserved WestBot

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/MoveAction.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.MoveAction" )
}

MoveAction::MoveAction(
    const TrajectoryManager::Ptr& trajectoryManager,
    TrajectoryManager::TrajectoryType type,
    float theta,
    float distance,
    float x,
    float y,
    bool correction,
    bool doNotBlock )
    : Action( "Move" )
    , _trajectoryManager( trajectoryManager )
    ,  _type( type )
    , _theta( theta )
    , _distance( distance )
    , _x( x )
    , _y( y )
    , _correction( correction )
    , _doNotBlock( doNotBlock )
{
}

void MoveAction::execute()
{
    tDebug( LOG ) << "Running" << name() << "action";

    switch( _type )
    {
    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_DISABLE:
        _trajectoryManager->disable();
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ENABLE:
        _trajectoryManager->enable();
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_STOP:
        _trajectoryManager->stop();
        break;

   case TrajectoryManager::TrajectoryType::TYPE_TRAJ_HARDSTOP:
        _trajectoryManager->hardStop();
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL:
        _trajectoryManager->moveDRel( _distance, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_D_REL:
        _trajectoryManager->moveOnlyDRel( _distance, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL:
        _trajectoryManager->turnARel( _theta, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_ABS:
        _trajectoryManager->turnAAbs( _theta, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_REL:
        _trajectoryManager->turnOnlyARel( _theta, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS:
        tDebug( LOG ) << "Move Only ABS" << _theta;

        _trajectoryManager->turnOnlyAAbs( _theta, _correction, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_A_REL:
        _trajectoryManager->moveToDARel(
            _theta, 
            _distance, 
            _correction, 
            _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY:
        tDebug( LOG ) << "Turn To XY" << _x << _y;
        _trajectoryManager->turnToXY( _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_TURNTO_XY_BEHIND:
        _trajectoryManager->turnToXYBehind( _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS:
        tDebug( LOG ) << "Move XY ABS" << _x << _y;
        _trajectoryManager->moveToXYAbs( _theta, _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS:
        tDebug( LOG ) << "Move FORWARD XY ABS" << _x << _y;
        _trajectoryManager->moveForwardToXYAbs( _theta, _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS:
        _trajectoryManager->moveBackwardToXYAbs( _theta, _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_D_A_REL:
        // TODO: XXX
        break;

    case TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_REL:
        _trajectoryManager->moveToXYRel( _x, _y, _doNotBlock );
        break;

    case TrajectoryManager::TrajectoryType::WINDOW_CONFIG:
        _trajectoryManager->waitTrajReady();
        break;

    case TrajectoryManager::TrajectoryType::WAIT_TRAJ_READY:
        _trajectoryManager->waitTrajReady();
        break;

    default:
        tWarning( LOG ) << "Not a valid trajectory type" << _type;
    }

    emit complete();
}
