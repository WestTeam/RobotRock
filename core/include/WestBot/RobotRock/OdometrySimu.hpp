// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ODOMETRYSIMU_HPP_
#define WESTBOT_ROBOTROCK_ODOMETRYSIMU_HPP_

#include <QMutex>

#include "Odometry.hpp"

namespace WestBot {
namespace RobotRock {

class OdometrySimu : public Odometry
{
public:
    OdometrySimu();
    ~OdometrySimu() override = default;

    RobotPos getPosition() override;
    void setPosition( RobotPos pos ) override;
    void addError( RobotPos error ) override;

private:
    RobotPos _posLatest;
    RobotPos _cumulatedError;


    QMutex _lock;
};

}
}

#endif // WESTBOT_ROBOTROCK_ODOMETRYSIMU_HPP_
