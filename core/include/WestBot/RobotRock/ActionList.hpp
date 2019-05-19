// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
#define WESTBOT_ROBOTROCK_ACTIONLIST_HPP_

#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/TrajectoryManager.hpp>
#include <WestBot/RobotRock/WaitAction.hpp>

namespace WestBot {
namespace RobotRock {

// TODO: A voir ce qu'on veux. Soit on un ensemble d'actions ici et on vient
// piocher dedans.
// Soit on a builder d'actions et la methode renvoie juste un bool avec le
// resultat de l'actions...

// Move actions
MoveAction::Ptr moveToCenterZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        inv * 500,
        600,
        true );
};

MoveAction::Ptr moveToStartZone(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_GOTO_FORWARD_XY_ABS,
        0.0,
        0.0,
        inv * 800,
        600,
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

MoveAction::Ptr moveALittleForward(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
        0.0,
        100.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr moveALittleForward2(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
        0.0,
        150.0,
        0.0,
        0.0,
        true );
}

MoveAction::Ptr turnA90(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_ONLY_A_ABS,
        90.0,
        0.0,
        0.0,
        0.0,
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

MoveAction::Ptr orientationZoneDepose(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_A_REL,
        20.0 * inv,
        0.0,
        0.0,
        0.0,
        true );
}

// Wait actions
WaitAction::Ptr wait500Ms()
{
    return std::make_shared< WaitAction >( 500 );
}

}
}

#endif // WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
