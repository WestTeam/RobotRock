// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
#define WESTBOT_ROBOTROCK_ACTIONLIST_HPP_

#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/TrajectoryManager.hpp>
#include <WestBot/RobotRock/WaitAction.hpp>

namespace WestBot {
namespace RobotRock {

// Move actions

MoveAction::Ptr moveToCenterZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS,
        0.0,
        0.0,
        600,
        inv * 500,
        true );
}

MoveAction::Ptr turnToCenterZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_REL,
        inv * 90.0,
        0.0,
        0,
        0,
        true );
}


MoveAction::Ptr moveALittleForward(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS,
        0.0,
        0.0,
        720,
        inv * 500,
        true );
}

MoveAction::Ptr moveALittleForward2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_XY_ABS,
        0.0,
        0.0,
        880,
        inv * 500,
        true );
}

MoveAction::Ptr moveToStartZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        600,
        inv * 800,
        true );
}

MoveAction::Ptr orientationZoneDepose(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
        -20.0 * inv,
        0.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr moveALittleForward3(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
        0.0,
        200.0,
        0,
        0,
        true );
}

/*MoveAction::Ptr moveToAccelerator(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        190,
        0,
        true );
}

MoveAction::Ptr alignToAccelerator(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS,
        0.0,
        0.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr moveToAccelerator2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        190,
        inv * -200,
        true );
}

MoveAction::Ptr moveToAccelerator3(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        190,
        inv * -500,
        true );
}

MoveAction::Ptr moveToBalance(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1200,
        inv * 150,
        true );

}

MoveAction::Ptr moveToBalance2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1400,
        inv * 150,
        true );
}

MoveAction::Ptr moveToCenterZone2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
        0.0,
        0.0,
        1050,
        inv * 150,
        true );
}

MoveAction::Ptr turnA90(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_REL,
        inv * -90.0,
        0.0,
        0.0,
        0.0,
        true );
}




MoveAction::Ptr moveSmall(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
        0.0,
        50.0,
        0.0,
        0.0,
        true );
}


MoveAction::Ptr turnR90(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_REL,
        inv * -90.0,
        0.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr turnA180(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS,
        inv * -180.0,
        0.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr onSaqueDedans(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1050,
        inv * 250,
        true );
}

MoveAction::Ptr onSaqueDedans2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1050,
        inv * 800,
        true );
}

MoveAction::Ptr goingBackALittle(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_BACKWARD_XY_ABS,
        0.0,
        0.0,
        1150,
        inv * 150,
        true );
}

MoveAction::Ptr getHighPucks(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1200,
        inv * 500,
        true );
}

MoveAction::Ptr getHighPucks2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        1350,
        inv * 500,
        true );
}

MoveAction::Ptr turnRedZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
        20.0,
        0.0,
        0.0,
        0.0,
        true );
}

*/
// Wait actions
WaitAction::Ptr wait500Ms()
{
    return std::make_shared< WaitAction >( 500 );
}

}
}

#endif // WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
