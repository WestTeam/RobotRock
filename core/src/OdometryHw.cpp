// Copyright (c) 2019 All Rights Reserved WestBot

#include <QMutexLocker>
#include <QThread>

#include <WestBot/HumanAfterAll/Category.hpp>

#include <WestBot/RobotRock/OdometryHw.hpp>

using namespace WestBot;
using namespace WestBot::RobotRock;

namespace
{
    HUMANAFTERALL_LOGGING_CATEGORY( LOG, "WestBot.RobotRock.OdometryHw" )
}

OdometryHw::OdometryHw( const Hal::Ptr& hal )
    : _hal( hal )
{
    _rd_pos_id = 0;
    _rd_pos_valid = 0;
}

// Get Latest position from Odometry core
RobotPos OdometryHw::getPosition()
{
    QMutexLocker locker( & _lock );
    RobotPos ret = _posLatest;

    uint8_t pos_id = _hal->_odometryId.read<uint8_t>();

    if (_hal->_odometryValid.read<uint8_t>() == 1)
    {
        // we check if there is a new position available, otherwise no need to update
        if (pos_id != _rd_pos_id || _rd_pos_valid == 0)
        {
            int16_t pos_teta = _hal->_odometryTheta.read<int16_t>();
            int16_t pos_x    = _hal->_odometryX.read<int16_t>();
            int16_t pos_y    = _hal->_odometryY.read<int16_t>();

            //tDebug( LOG ) << pos_id << pos_teta << pos_x << pos_y;

            // we check again the pos_id is the same as well as valid, to make sure the data we read is ok.
            if (_hal->_odometryId.read<uint8_t>() == pos_id && _hal->_odometryValid.read<uint8_t>() == 1)
            {
                ret.x = pos_x;
                ret.y = pos_y;
                ret.theta = RAD(((double)pos_teta)/100.0);
                //tDebug( LOG ) << pos_teta << pos_x << pos_y << ret.theta << ((double)pos_teta)/100.0;

                _posLatest = ret;

                _rd_pos_valid = 1;
                _rd_pos_id = pos_id;

            }else{
                tDebug( LOG ) << "getPosition() => position changed during read";
            }
        }
    } else {
        //tDebug( LOG ) << "getPosition() => no valid position" << _hal->_odometryValid.read<uint8_t>() << _rd_pos_valid << pos_id << _rd_pos_id;
    }

    return ret;
}

void OdometryHw::setPosition( RobotPos pos )
{
    // first we read current position
    RobotPos curPos = this->getPosition();
    RobotPos error;

    error.x = pos.x-curPos.x;
    error.y = pos.y-curPos.y;
    error.theta = pos.theta-curPos.theta;

    addError( error );
}

void OdometryHw::addError( RobotPos pos )
{
    QMutexLocker locker( & _lock );

    uint8_t commandId = _hal->_odometryAck.read< uint8_t >();

    int16_t x = (int16_t)round(pos.x);
    int16_t y = (int16_t)round(pos.y);
    int16_t theta = (int16_t)round(DEG(pos.theta)*100.0);

    _hal->_odometryValid.write(0);
    _hal->_odometryId.write(commandId++);
    _hal->_odometryTheta.write(theta);
    _hal->_odometryX.write(x);
    _hal->_odometryY.write(y);
    _hal->_odometryValid.write(1);

    _wr_pos_id = commandId;

    while( _hal->_odometryAck.read< uint8_t >() != commandId )
    {
        QThread::msleep( 1 );
        tDebug( LOG ) << "wait cmd ack";
    }
}
