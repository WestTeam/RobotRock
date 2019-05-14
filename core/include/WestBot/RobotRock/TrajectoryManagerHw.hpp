// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_TRAJECTORYMANAGERHW_HPP_
#define WESTBOT_ROBOTROCK_TRAJECTORYMANAGERHW_HPP_

#include "Hal.hpp"
#include "TrajectoryManager.hpp"

#define CMD_TYPE_TRAJ 0x0
#define CMD_TYPE_CFG_DISTANCE 0x1
#define CMD_TYPE_CFG_ANGLE 0x2
#define CMD_TYPE_CFG_WND 0x3

#define TRAJ_DISABLE 0
#define TRAJ_ENABLE 1
#define TRAJ_STOP 2
#define TRAJ_HARDSTOP 3
#define TRAJ_D_REL 4
#define TRAJ_ONLY_D_REL 5
#define TRAJ_A_REL 6
#define TRAJ_A_ABS 7
#define TRAJ_ONLY_A_REL 8
#define TRAJ_ONLY_A_ABS 9
#define TRAJ_D_A_REL 10
#define TRAJ_TURNTO_XY 11
#define TRAJ_TURNTO_XY_BEHIND 12
#define TRAJ_GOTO_XY_ABS 13
#define TRAJ_GOTO_FORWARD_XY_ABS 14
#define TRAJ_GOTO_BACKWARD_XY_ABS 15
#define TRAJ_GOTO_D_A_REL 16
#define TRAJ_GOTO_XY_REL 17

namespace WestBot {
namespace RobotRock {

class TrajectoryManagerHw : public TrajectoryManager
{
public:
    TrajectoryManagerHw( const Hal::Ptr& hal );

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
    Hal::Ptr _hal;
    bool _abort;
};

}
}

#endif // WESTBOT_ROBOTROCK_TRAJECTORYMANAGEHWR_HPP_
