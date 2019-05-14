// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_TRAJECTORYMANAGERSIMU_HPP_
#define WESTBOT_ROBOTROCK_TRAJECTORYMANAGERSIMU_HPP_

#include <QThread>


#include "Hal.hpp"
#include "TrajectoryManager.hpp"
#include "Odometry.hpp"


namespace WestBot {
namespace RobotRock {

class TrajectoryManagerSimu : public TrajectoryManager, QThread
{
public:
    TrajectoryManagerSimu( const Odometry::Ptr& odometry );

    void init() override;

    void waitTrajReady() override;
    bool isTrajReady() override;

    void disable() override;
    void enable() override;

    void stop() override;
    void hardStop() override;

    void setAbort( bool abort ) override;

    void setDistanceConfig( float speed, float acc ) override;
    void setAngleConfig( float speed, float acc ) override;

    void setWindow(
        float distance,
        float angleDeg,
        float startAngleDeg ) override;

    void moveDRel(
        float distance,
        bool correction,
        bool doNotBlock = false ) override;

    void moveOnlyDRel(
        float distance,
        bool correction,
        bool doNotBlock = false ) override;

    void turnARel(
        float theta,
        bool correction,
        bool doNotBlock = false ) override;

    void turnAAbs(
        float theta,
        bool correction,
        bool doNotBlock = false ) override;

    void turnOnlyARel(
        float theta,
        bool correction,
        bool doNotBlock = false ) override;

    void turnOnlyAAbs(
        float theta,
        bool correction,
        bool doNotBlock = false ) override;

    void turnToXY(
        float x,
        float y,
        bool doNotBlock = false ) override;

    void turnToXYBehind(
        float x,
        float y,
        bool doNotBlock = false ) override;

    void moveToXYAbs(
        float theta,
        float x,
        float y,
        bool doNotBlock = false ) override;

    void moveForwardToXYAbs(
        float theta,
        float x,
        float y,
        bool doNotBlock = false ) override;

    void moveBackwardToXYAbs(
        float theta,
        float x,
        float y,
        bool doNotBlock = false ) override;

    void moveToDARel(
        float theta,
        float distance,
        bool correction,
        bool doNotBlock = false ) override;

    void moveToXYRel( float x, float y, bool doNotBlock = false ) override;

private:
    void run() override;

private:
    enum TrajectoryType _trajType;
    enum TrajectoryState _trajState;
    uint8_t _commandId;
    float _commandDistance;
    float _commandAngle;
    float _commandX;
    float _commandY;





    bool _enabled;

    float _wndDist;
    float _wndAngleDeg;
    float _wndStartAngleDeg;

    float _distSpeed;
    float _distAcc;
    float _angleSpeed;
    float _angleAcc;



    RobotPos _targetPos;

    Odometry::Ptr _odometry;
    bool _abort;
};

}
}

#endif // WESTBOT_ROBOTROCK_TRAJECTORYMANAGERSIMU_HPP_
