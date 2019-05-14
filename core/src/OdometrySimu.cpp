// Copyright (c) 2019 All Rights Reserved WestBot

#include <QMutexLocker>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/OdometrySimu.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.OdometrySimu" )
}

OdometrySimu::OdometrySimu()
{
    _cumulatedError.x = 0.0;
    _cumulatedError.y = 0.0;
    _cumulatedError.theta = 0.0;

}

// Get Latest position from Odometry core
RobotPos OdometrySimu::getPosition()
{
    QMutexLocker locker( & _lock );

    return _cumulatedError;
}

void OdometrySimu::setPosition( RobotPos pos )
{
    // first we read current position
    RobotPos curPos = this->getPosition();
    RobotPos error;

    error.x = pos.x-curPos.x;
    error.y = pos.y-curPos.y;
    error.theta = pos.theta-curPos.theta;

    addError( error );
}

void OdometrySimu::addError( RobotPos pos )
{
    QMutexLocker locker( & _lock );

    _cumulatedError.x += pos.x;
    _cumulatedError.y += pos.y;
    _cumulatedError.theta += pos.theta;
}