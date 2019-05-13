// Copyright (c) 2019 All Rights Reserved WestBot

#ifndef WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
#define WESTBOT_ROBOTROCK_ACTIONLIST_HPP_

#include <WestBot/RobotRock/MoveAction.hpp>
#include <WestBot/RobotRock/TrajectoryManager.hpp>

namespace WestBot {
namespace RobotRock {

// TODO: A voir ce qu'on veux. Soit on un ensemble d'actions ici et on vient
// piocher dedans.
// Soit on a builder d'actions et la methode renvoie juste un bool avec le
// resultat de l'actions...

// Move actions
MoveAction::Ptr moveToFirstPuck(
    const TrajectoryManager::Ptr& trajectoryManager,
    float inv )
{
    return std::make_shared< MoveAction >(
        trajectoryManager,
        TrajectoryManager::TrajectoryType::TYPE_TRAJ_D_REL,
        0.0,
        100000.0,
        0.0,
        0.0 * inv,
        true );


}

// Wait actions
WaitAction::Ptr wait500Ms()
{
    return std::make_shared< WaitAction >( 500 );
}

};

}
}

#endif // WESTBOT_ROBOTROCK_ACTIONLIST_HPP_
