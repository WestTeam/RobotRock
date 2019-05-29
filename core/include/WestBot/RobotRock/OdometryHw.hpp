// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ODOMETRYHW_HPP_
#define WESTBOT_ROBOTROCK_ODOMETRYHW_HPP_

#include <QMutex>

#include "Hal.hpp"
#include "Odometry.hpp"

namespace WestBot {
namespace RobotRock {

class OdometryHw : public Odometry
{
public:
    OdometryHw( const Hal::Ptr& hal );
    ~OdometryHw() override = default;

    RobotPos getPosition() override;
    void setPosition( RobotPos pos ) override;
    void addError( RobotPos error ) override;

private:
    Hal::Ptr _hal;
    RobotPos _posLatest;
    // for pos reading
    uint8_t _rd_pos_id;
    uint8_t _rd_pos_valid;
    // for pos writing
    uint8_t _wr_pos_id;
    uint8_t _wr_pos_valid;

    QMutex _lock;
};

}
}

#endif // WESTBOT_ROBOTROCK_ODOMETRYHW_HPP_
